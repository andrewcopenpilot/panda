/*
powertrain: 0x24B: Debug messages
powertrain: 0xF1: body control module
powertrain: 0xC9: Engine contorl module
powertrain: 0x1E9: brake control module
powertrain: 0x1C4: Engine Control Module
powertrain: 0x1C5: Engine control module
powertrain: 0x1F5: Telematic control module
powertrain: 0x1E1: Body Control Module
powertrain: 0x214: brake control module
powertrain: 0x230: electronic parking brake
powertrain: 0x34A: brake control module
powertrain: 0x12A: body control module
powertrain: 0x135: body control module
powertrain: 0x184: power steering control module
powertrain: 0x1F1: body control module
powertrain: 0x140: body control module

powertrain: 0x17D appears to be related to traction control status

Powertrain: 0xBD (Volt Specific) Hybrid/EV Powertrain control module

chassis: 0xC1: electronic brake control module
chassis: 0xC5: electronic brake control module
chassis: 0x130: multi axis accelerometer
chassis: 0x140: multi axis accelerometer
chassis: 0x170: electronic brake control module
chassis: 0x1E5: steering angle sensor
chassis: 0xC0: Seems to be needed for auto highbeams

object: 0x180 lkasteeringcmd proxy
object: 0x2CB gas regen proxy
object: 0x370 ASCMActiveCruiseControlStatus proxy
*/

#define MIN(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a < _b) ? _a : _b; })

#define MAX(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a > _b) ? _a : _b; })

#define GET_BUS(msg) (((msg)->RDTR >> 4) & 0xFF)

#define GET_ADDR(msg) ((((msg)->RIR & 4) != 0) ? ((msg)->RIR >> 3) : ((msg)->RIR >> 21))
#define GET_BYTE(msg, b) (((int)(b) > 3) ? (((msg)->RDHR >> (8U * ((unsigned int)(b) % 4U))) & 0XFFU) : (((msg)->RDLR >> (8U * (unsigned int)(b))) & 0xFFU))

// IRQs: CAN1_TX, CAN1_RX0, CAN1_SCE, CAN2_TX, CAN2_RX0, CAN2_SCE, CAN3_TX, CAN3_RX0, CAN3_SCE

#define CAN_MAX 3

#define can_buffer(x, size) \
  CAN_FIFOMailBox_TypeDef elems_##x[size]; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = size, .elems = (CAN_FIFOMailBox_TypeDef *)&elems_##x };

can_buffer(tx1_q, 0x100)
can_buffer(tx2_q, 0x100)
can_buffer(tx3_q, 0x100)
can_buffer(txgmlan_q, 0x100)
can_ring *can_queues[] = {&can_tx1_q, &can_tx2_q, &can_tx3_q, &can_txgmlan_q};

// params and flags about checksum, counter and frequency checks for each monitored address
typedef struct {
  // const params
  const int addr[3];                 // check either messages (e.g. honda steer). Array MUST terminate with a zero to know its length.
  const int bus;                     // bus where to expect the addr. Temp hack: -1 means skip the bus check
  const bool check_checksum;         // true is checksum check is performed
  const uint8_t max_counter;         // maximum value of the counter. 0 means that the counter check is skipped
  const uint32_t expected_timestep;  // expected time between message updates [us]
  // dynamic flags
  bool valid_checksum;               // true if and only if checksum check is passed
  int wrong_counters;                // counter of wrong counters, saturated between 0 and MAX_WRONG_COUNTERS
  uint8_t last_counter;              // last counter value
  uint32_t last_timestamp;           // micro-s
  bool lagging;                      // true if and only if the time between updates is excessive
} AddrCheckStruct;

// sample struct that keeps 3 samples in memory
struct sample_t {
  int values[6];
  int min;
  int max;
} sample_t_default = {{0}, 0, 0};

// convert a trimmed integer to signed 32 bit int
int to_signed(int d, int bits) {
  int d_signed = d;
  if (d >= (1 << MAX((bits - 1), 0))) {
    d_signed = d - (1 << MAX(bits, 0));
  }
  return d_signed;
}

// given a new sample, update the smaple_t struct
void update_sample(struct sample_t *sample, int sample_new) {
  int sample_size = sizeof(sample->values) / sizeof(sample->values[0]);
  for (int i = sample_size - 1; i > 0; i--) {
    sample->values[i] = sample->values[i-1];
  }
  sample->values[0] = sample_new;

  // get the minimum and maximum measured samples
  sample->min = sample->values[0];
  sample->max = sample->values[0];
  for (int i = 1; i < sample_size; i++) {
    if (sample->values[i] < sample->min) {
      sample->min = sample->values[i];
    }
    if (sample->values[i] > sample->max) {
      sample->max = sample->values[i];
    }
  }
}

bool max_limit_check(int val, const int MAX_VAL, const int MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

// real time check, mainly used for steer torque rate limiter
bool rt_rate_limit_check(int val, int val_last, const int MAX_RT_DELTA) {

  // *** torque real time rate limit check ***
  int highest_val = MAX(val_last, 0) + MAX_RT_DELTA;
  int lowest_val = MIN(val_last, 0) - MAX_RT_DELTA;

  // check for violation
  return (val < lowest_val) || (val > highest_val);
}

// compute the time elapsed (in microseconds) from 2 counter samples
// case where ts < ts_last is ok: overflow is properly re-casted into uint32_t
uint32_t get_ts_elapsed(uint32_t ts, uint32_t ts_last) {
  return ts - ts_last;
}

int get_addr_check_index(CAN_FIFOMailBox_TypeDef *to_push, AddrCheckStruct addr_list[], const int len) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  int index = -1;
  for (int i = 0; i < len; i++) {
    for (uint8_t j = 0U; addr_list[i].addr[j] != 0; j++) {
      if ((addr == addr_list[i].addr[j]) && (bus == addr_list[i].bus)) {
        index = i;
        goto Return;
      }
    }
  }
Return:
  return index;
}

// check that commanded value isn't fighting against driver
bool driver_limit_check(int val, int val_last, struct sample_t *val_driver,
  const int MAX_VAL, const int MAX_RATE_UP, const int MAX_RATE_DOWN,
  const int MAX_ALLOWANCE, const int DRIVER_FACTOR) {

  int highest_allowed_rl = MAX(val_last, 0) + MAX_RATE_UP;
  int lowest_allowed_rl = MIN(val_last, 0) - MAX_RATE_UP;

  int driver_max_limit = MAX_VAL + (MAX_ALLOWANCE + val_driver->max) * DRIVER_FACTOR;
  int driver_min_limit = -MAX_VAL + (-MAX_ALLOWANCE + val_driver->min) * DRIVER_FACTOR;

  // if we've exceeded the applied torque, we must start moving toward 0
  int highest_allowed = MIN(highest_allowed_rl, MAX(val_last - MAX_RATE_DOWN,
                                             MAX(driver_max_limit, 0)));
  int lowest_allowed = MAX(lowest_allowed_rl, MIN(val_last + MAX_RATE_DOWN,
                                           MIN(driver_min_limit, 0)));

  // check for violation
  return (val < lowest_allowed) || (val > highest_allowed);
}

const int GM_MAX_STEER = 300;
const int GM_MAX_RT_DELTA = 128;          // max delta torque allowed for real time checks
const uint32_t GM_RT_INTERVAL = 250000;    // 250ms between real time checks
const int GM_MAX_RATE_UP = 7;
const int GM_MAX_RATE_DOWN = 17;
const int GM_DRIVER_TORQUE_ALLOWANCE = 50;
const int GM_DRIVER_TORQUE_FACTOR = 4;
const int GM_MAX_GAS = 4500;
const int GM_MAX_REGEN = 1404;

int gm_rt_torque_last = 0;
int gm_desired_torque_last = 0;
uint32_t gm_ts_last = 0;
struct sample_t gm_torque_driver;         // last few driver torques measured

int can_err_cnt = 0;
int can0_mailbox_full_cnt = 0;
int can1_mailbox_full_cnt = 0;
int can2_mailbox_full_cnt = 0;

int can0_rx_cnt = 0;
int can1_rx_cnt = 0;
int can2_rx_cnt = 0;

int can0_tx_cnt = 0;
int can1_tx_cnt = 0;
int can2_tx_cnt = 0;

uint32_t tick = 0;
uint8_t ascm_acc_cmd_active = 0;

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number);
bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem);
void send_interceptor_status();
void send_steering_msg(uint32_t tick);

CAN_FIFOMailBox_TypeDef steering_oem;
volatile int steering_oem_ttl = 0;

uint8_t steering_violation_cnt = 0;

// overrides
CAN_FIFOMailBox_TypeDef steering_override;
volatile int steering_override_ttl = 0;
volatile uint8_t steering_rolling_counter = 0;

CAN_FIFOMailBox_TypeDef gas_regen_override;
volatile int gas_regen_override_ttl = 0;
int gas_regen_counter;

CAN_FIFOMailBox_TypeDef acc_status_override;
volatile int acc_status_ttl = 0;


// assign CAN numbering
// bus num: Can bus number on ODB connector. Sent to/from USB
//    Min: 0; Max: 127; Bit 7 marks message as receipt (bus 129 is receipt for but 1)
// cans: Look up MCU can interface from bus number
// can number: numeric lookup for MCU CAN interfaces (0 = CAN1, 1 = CAN2, etc);
// bus_lookup: Translates from 'can number' to 'bus number'.
// can_num_lookup: Translates from 'bus number' to 'can number'.
// can_forwarding: Given a bus num, lookup bus num to forward to. -1 means no forward.

// Panda:       Bus 0=CAN1   Bus 1=CAN2   Bus 2=CAN3
CAN_TypeDef *cans[] = {CAN1, CAN2, CAN3};
uint8_t bus_lookup[] = {0,1,2};
uint8_t can_num_lookup[] = {0,1,2,-1};
int8_t can_forwarding[] = {-1,-1,-1,-1};
uint32_t can_speed[] = {5000, 5000, 5000, 333};

#define AUTOBAUD_SPEEDS_LEN (sizeof(can_autobaud_speeds) / sizeof(can_autobaud_speeds[0]))

#define CANIF_FROM_CAN_NUM(num) (cans[num])
#ifdef PANDA
#define CAN_NUM_FROM_CANIF(CAN) (CAN==CAN1 ? 0 : (CAN==CAN2 ? 1 : 2))
#define CAN_NAME_FROM_CANIF(CAN) (CAN==CAN1 ? "CAN1" : (CAN==CAN2 ? "CAN2" : "CAN3"))
#else
#define CAN_NUM_FROM_CANIF(CAN) (CAN==CAN1 ? 0 : 1)
#define CAN_NAME_FROM_CANIF(CAN) (CAN==CAN1 ? "CAN1" : "CAN2")
#endif
#define BUS_NUM_FROM_CAN_NUM(num) (bus_lookup[num])
#define CAN_NUM_FROM_BUS_NUM(num) (can_num_lookup[num])

// other option
/*#define CAN_QUANTA 16
#define CAN_SEQ1 13
#define CAN_SEQ2 2*/

// this is needed for 1 mbps support
#define CAN_QUANTA 8
#define CAN_SEQ1 6 // roundf(quanta * 0.875f) - 1;
#define CAN_SEQ2 1 // roundf(quanta * 0.125f);

#define CAN_PCLK 24000
// 333 = 33.3 kbps
// 5000 = 500 kbps
#define can_speed_to_prescaler(x) (CAN_PCLK / CAN_QUANTA * 10 / (x))

void can_set_speed(uint8_t can_number) {
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  while (true) {
    // initialization mode
    CAN->MCR = CAN_MCR_TTCM | CAN_MCR_INRQ;
    while((CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);

    // set time quanta from defines
    CAN->BTR = (CAN_BTR_TS1_0 * (CAN_SEQ1-1)) |
              (CAN_BTR_TS2_0 * (CAN_SEQ2-1)) |
              (can_speed_to_prescaler(can_speed[bus_number]) - 1);

    // reset
    CAN->MCR = CAN_MCR_TTCM | CAN_MCR_ABOM;

    #define CAN_TIMEOUT 1000000
    int tmp = 0;
    while((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK && tmp < CAN_TIMEOUT) tmp++;
    if (tmp < CAN_TIMEOUT) {
      return;
    }

    puts("CAN init FAILED!!!!!\n");
    puth(can_number); puts(" ");
    puth(BUS_NUM_FROM_CAN_NUM(can_number)); puts("\n");
    return;
  }
}

void process_can(uint8_t can_number) {
  if (can_number != 0xff) {

    enter_critical_section();

    CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
    uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

    // check for empty mailbox
    CAN_FIFOMailBox_TypeDef to_send;
    if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
      // add successfully transmitted message to my fifo
      if ((CAN->TSR & CAN_TSR_RQCP0) == CAN_TSR_RQCP0) {
        if ((CAN->TSR & CAN_TSR_TERR0) == CAN_TSR_TERR0) {
          #ifdef DEBUG
            puts("CAN TX ERROR!\n");
          #endif
        }

        if ((CAN->TSR & CAN_TSR_ALST0) == CAN_TSR_ALST0) {
          #ifdef DEBUG
            puts("CAN TX ARBITRATION LOST!\n");
          #endif
        }

        // clear interrupt
        // careful, this can also be cleared by requesting a transmission
        CAN->TSR |= CAN_TSR_RQCP0;
      }

      if (can_pop(can_queues[bus_number], &to_send)) {
        // only send if we have received a packet
        CAN->sTxMailBox[0].TDLR = to_send.RDLR;
        CAN->sTxMailBox[0].TDHR = to_send.RDHR;
        CAN->sTxMailBox[0].TDTR = to_send.RDTR;
        CAN->sTxMailBox[0].TIR = to_send.RIR;

	if (can_number == 0) {
          can0_tx_cnt++;
        }
	if (can_number == 1) {
	  can1_tx_cnt++;
	}
        if (can_number == 2) {
          can2_tx_cnt++;
        }
      }
    }

    exit_critical_section();
  }
}

void can_init_all() {
  // Wait for INAK bit to be set
  while(((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)) {}
  while(((CAN2->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)) {}
  while(((CAN3->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)) {}

  // filter master register - Set filter init mode
  CAN1->FMR |= CAN_FMR_FINIT;

  // Assign 14 filter banks to CAN 1 and 2 each
  CAN1->FMR |= (((uint32_t) 14) << CAN_FMR_CAN2SB_Pos) & CAN_FMR_CAN2SB_Msk;
 
  // filter mode register - CAN_FM1R_FBMX bit sets the associated filter bank to list mode. Only message IDs listed will be pushed to the rx fifo (vs ID mask mode)
  CAN1->FM1R = 0x00000000; // FM1R reset value
  CAN1->FM1R |= CAN_FM1R_FBM0 | CAN_FM1R_FBM1 | CAN_FM1R_FBM2 | CAN_FM1R_FBM3 | CAN_FM1R_FBM4 | CAN_FM1R_FBM5 | CAN_FM1R_FBM6 | CAN_FM1R_FBM7 | CAN_FM1R_FBM8 | CAN_FM1R_FBM14 | CAN_FM1R_FBM15 | CAN_FM1R_FBM16; 
  
  // filter scale register - Set all filter banks to be dual 16-bit (vs 32 bit)
  CAN1->FS1R = 0x00000000; // Reset value (all 16 bit mode)
  CAN1->FS1R = 0x0FFFFFFF; // Set all 28 filter banks to 32 bit mode

  // filter FIFO assignment register - Set all filters to store in FIFO 0 
  CAN1->FFA1R = 0x00000000;

  // filter activation register - CAN_FA1R_FACTX bit activate the associated filter bank
  CAN1->FA1R = 0x00000000; // Reset value

  CAN1->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 | CAN_FA1R_FACT2 | CAN_FA1R_FACT3 | CAN_FA1R_FACT4 | CAN_FA1R_FACT5 | CAN_FA1R_FACT6 | CAN_FA1R_FACT7 | CAN_FA1R_FACT8 | CAN_FA1R_FACT14 | CAN_FA1R_FACT15 | CAN_FA1R_FACT16;

  //Set CAN 1 Filters CAR PT
  CAN1->sFilterRegister[0].FR1 = 0x24B<<21;
  CAN1->sFilterRegister[0].FR2 = 0xF1<<21;
  CAN1->sFilterRegister[1].FR1 = 0xC9<<21;
  CAN1->sFilterRegister[1].FR2 = 0x1E9<<21;
  CAN1->sFilterRegister[2].FR1 = 0x1C4<<21;
  CAN1->sFilterRegister[2].FR2 = 0x1C5<<21;
  CAN1->sFilterRegister[3].FR1 = 0x1F5<<21;
  CAN1->sFilterRegister[3].FR2 = 0x1E1<<21;
  CAN1->sFilterRegister[4].FR1 = 0x214<<21;
  CAN1->sFilterRegister[4].FR2 = 0x230<<21;
  CAN1->sFilterRegister[5].FR1 = 0x34A<<21;
  CAN1->sFilterRegister[5].FR2 = 0x12A<<21;
  CAN1->sFilterRegister[6].FR1 = 0x135<<21;
  CAN1->sFilterRegister[6].FR2 = 0x184<<21;
  CAN1->sFilterRegister[7].FR1 = 0x1F1<<21;
  CAN1->sFilterRegister[7].FR2 = 0x140<<21;
  CAN1->sFilterRegister[8].FR1 = 0x17D<<21;
  CAN1->sFilterRegister[8].FR2 = 0xBD<<21;

  // Set Can 2 Filters Obj
  CAN1->sFilterRegister[14].FR1 = 0x180<<21; // lkasteeringcmd proxy 
  CAN1->sFilterRegister[14].FR2 = 0x2CB<<21; // gasregencmd proxy
  CAN1->sFilterRegister[15].FR1 = 0x370<<21; // ASCMActiveCruiseControlStatus proxy
  CAN1->sFilterRegister[15].FR2 = 0x376<<21; // Interceptor Chas status
  CAN1->sFilterRegister[16].FR1 = 0x377<<21; // Proxy SW GMLAN status
  CAN1->sFilterRegister[16].FR2 = 0x377<<21; 

  CAN1->FMR &= ~(CAN_FMR_FINIT);

  // filter master register - Set filter init mode
  CAN3->FMR |= CAN_FMR_FINIT;

  // filter mode register - Set all filter banks to mask mode
  CAN3->FM1R = 0x00000000; // Reset value

  // filter scale register - Set all filter banks to be dual 16-bit (vs 32 bit)
  CAN3->FS1R = 0x00000000; // Reset value (all 16 bit mode)
  CAN3->FS1R = 0x00003FFF; // Set all 14 filter banks to 32 bit mode

  // filter FIFO assignment register - Set all filters to store in FIFO 0
  CAN3->FFA1R = 0x00000000; // Reset value

  // filter activation register - CAN_FA1R_FACTX bit activate the associated filter bank
  CAN3->FA1R = 0x00000000; // Reset value
  CAN3->FA1R |= CAN_FA1R_FACT0;

  // Filter bank registers - A mask of 0 accepts all message IDs and stores in the associated FIFO
  CAN3->sFilterRegister[0].FR1 = 0;
  CAN3->sFilterRegister[0].FR2 = 0;
 
  CAN3->FMR &= ~(CAN_FMR_FINIT);

  // enable certain CAN interrupts
  CAN1->IER |= CAN_IER_TMEIE | CAN_IER_FMPIE0;
  CAN2->IER |= CAN_IER_TMEIE | CAN_IER_FMPIE0;
  CAN3->IER |= CAN_IER_TMEIE | CAN_IER_FMPIE0;

  NVIC_EnableIRQ(CAN1_TX_IRQn);
  NVIC_EnableIRQ(CAN1_RX0_IRQn);
  NVIC_EnableIRQ(CAN1_SCE_IRQn);
  NVIC_EnableIRQ(CAN2_TX_IRQn);
  NVIC_EnableIRQ(CAN2_RX0_IRQn);
  NVIC_EnableIRQ(CAN2_SCE_IRQn);
  NVIC_EnableIRQ(CAN3_TX_IRQn);
  NVIC_EnableIRQ(CAN3_RX0_IRQn);
  NVIC_EnableIRQ(CAN3_SCE_IRQn);

  set_can_enable(CAN1, 1);
  set_can_enable(CAN2, 1);
  set_can_enable(CAN3, 1);
  can_set_speed(0);
  can_set_speed(1);
  can_set_speed(2);

  // in case there are queued up messages
  process_can(0);
  process_can(1);
  process_can(2);
}

int can_overflow_cnt = 0;

// ********************* interrupt safe queue *********************

bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
  bool ret = 0;

  enter_critical_section();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1) == q->fifo_size) q->r_ptr = 0;
    else q->r_ptr += 1;
    ret = 1;
  }
  exit_critical_section();

  return ret;
}

int can_push(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
  int ret = 0;
  uint32_t next_w_ptr;

  enter_critical_section();
  if ((q->w_ptr + 1) == q->fifo_size) next_w_ptr = 0;
  else next_w_ptr = q->w_ptr + 1;
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = 1;
  }
  exit_critical_section();
  if (ret == 0) {
    can_overflow_cnt++;
    #ifdef DEBUG
      puts("can_push failed!\n");
    #endif
  }
  return ret;
}

// CAN error
void can_sce(CAN_TypeDef *CAN) {
  enter_critical_section();

  can_err_cnt += 1;
  if (CAN==CAN1) puts("CAN1:  ");
  if (CAN==CAN2) puts("CAN2:  ");
  if (CAN==CAN3) puts("CAN3:  ");
  puts("MSR:");
  puth(CAN->MSR);
  puts(" TSR:");
  puth(CAN->TSR);
  puts(" RF0R:");
  puth(CAN->RF0R);
  puts(" RF1R:");
  puth(CAN->RF1R);
  puts(" ESR:");
  puth(CAN->ESR);
  puts("\n");

  // clear current send
  CAN->TSR |= CAN_TSR_ABRQ0;
  CAN->MSR &= ~(CAN_MSR_ERRI);
  CAN->MSR = CAN->MSR;

  exit_critical_section();
}

void ttl_timer_init() {
  TIM3->PSC = 4800-1;	        // Set prescaler to 24 000 (PSC + 1)
  TIM3->ARR = 200-1;	          // Auto reload value 1000
  TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
  TIM3->CR1 = TIM_CR1_CEN;   // Enable timer

  NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
if (TIM3->SR & TIM_SR_UIF) // if UIF flag is set
  {
    TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
    enter_critical_section();
    if (steering_oem_ttl > 0) {
        steering_oem_ttl--;
    }
    if (steering_override_ttl > 0) {
        steering_override_ttl--;
    }
    if (gas_regen_override_ttl > 0) {
        gas_regen_override_ttl--;
    }
    if (acc_status_ttl > 0) {
        acc_status_ttl--;
    }
    exit_critical_section();

    tick++;
    if (tick % 5 == 0) {
        send_interceptor_status();
    }
    send_steering_msg(tick);
  }
}

// ***************************** CAN *****************************

int RESUME_MSG[4] = {0xBF2C00, 0xEE2101, 0xDD2602, 0xCC2B03};
int UNPRESS_MSG[4] = {0xFF1000, 0xEE1501, 0xDD1A02, 0xCC1F03};
int CRUISE_MAIN_MSG[4] = {0xBF5000, 0xAE5501, 0x9D5A02, 0x8C5F03};

void send_interceptor_status() {
    CAN_FIFOMailBox_TypeDef status;

    status.RIR = (885 << 21) | 1;
    status.RDTR = 8;
    status.RDLR = 0;
    status.RDLR |= ascm_acc_cmd_active;
    status.RDHR = 0xFFFFFFFF | steering_violation_cnt;

    can_push(can_queues[1], &status);
    process_can(CAN_NUM_FROM_BUS_NUM(1));
}	

void send_steering_msg(uint32_t tick) {
    CAN_FIFOMailBox_TypeDef steer;
    steer.RIR = (0x180 << 21) | 1;

    if (steering_override_ttl > 0) {
    	steer.RDTR = steering_override.RDTR;
    	steer.RDLR = steering_override.RDLR;
    	steer.RDHR = steering_override.RDHR;
    }
    else if (steering_oem_ttl > 0) {
        steer.RDTR = steering_oem.RDTR;
	steer.RDLR = steering_oem.RDLR;
	steer.RDHR = steering_oem.RDHR;
    }
    else {
        return;
    }

    // Only update at 10 Hz when not active
    uint8_t active = (steer.RDLR >> 3) & 0x1U;
    if (!active && (tick % 5 != 0))
        return;

    // Pull out LKA Steering CMD data and swap endianness (not including rolling counter)
    uint32_t dataswap = ((steer.RDLR << 8) & 0x0F00U) | ((steer.RDLR >> 8) &0xFFU);
    uint32_t checksum = (0x1000 - dataswap - steering_rolling_counter) & 0x0fff;

    //Swap endianness of checksum back to what GM expects
    uint32_t checksumswap = (checksum >> 8) | ((checksum << 8) & 0xFF00U);

    // Merge the rewritten checksum back into the BxCAN frame RDLR
    steer.RDLR = (steer.RDLR & 0x0000FFCF) | (checksumswap << 16) | (steering_rolling_counter << 4);

    steering_rolling_counter++;
    steering_rolling_counter %= 4;

    // LKA STEER: safety check
    int desired_torque = ((GET_BYTE(&steer, 0) & 0x7U) << 8) + GET_BYTE(&steer, 1);
    //uint32_t ts = TIM2->CNT;
    bool violation = 0;
    desired_torque = to_signed(desired_torque, 11);

    // *** global torque limit check ***
    violation |= max_limit_check(desired_torque, GM_MAX_STEER, -GM_MAX_STEER);

    // *** torque rate limit check ***
    //violation |= driver_limit_check(desired_torque, gm_desired_torque_last, &gm_torque_driver,
    //  GM_MAX_STEER, GM_MAX_RATE_UP, GM_MAX_RATE_DOWN,
    //  GM_DRIVER_TORQUE_ALLOWANCE, GM_DRIVER_TORQUE_FACTOR);

    // used next time
    //gm_desired_torque_last = desired_torque;

    // *** torque real time rate limit check ***
    //violation |= rt_rate_limit_check(desired_torque, gm_rt_torque_last, GM_MAX_RT_DELTA);

    // every RT_INTERVAL set the new limits
    //uint32_t ts_elapsed = get_ts_elapsed(ts, gm_ts_last);
    //if (ts_elapsed > GM_RT_INTERVAL) {
    //  gm_rt_torque_last = desired_torque;
    //  gm_ts_last = ts;
    //}
    
    // reset to 0 if there's a violation
    //if (violation || (desired_torque == 0)) {
    //  gm_desired_torque_last = 0;
    //  gm_rt_torque_last = 0;
    //  gm_ts_last = ts;
    //}

    if (violation) {
      steering_violation_cnt++;
      return;
    }

    can_push(can_queues[0], &steer);
    process_can(CAN_NUM_FROM_BUS_NUM(0));
}

void handle_update_steering_override(CAN_FIFOMailBox_TypeDef *override_msg) {
    steering_override.RIR = override_msg->RIR;
    steering_override.RDTR = override_msg->RDTR;
    steering_override.RDLR = override_msg->RDLR;
    steering_override.RDHR = override_msg->RDHR;

    steering_override_ttl = 50;
}

void handle_update_steering_oem(CAN_FIFOMailBox_TypeDef *oem_msg) {
    steering_oem.RIR =  oem_msg->RIR;
    steering_oem.RDTR = oem_msg->RDTR;
    steering_oem.RDLR = oem_msg->RDLR;
    steering_oem.RDHR = oem_msg->RDHR;

    steering_oem_ttl = 50;
}

void handle_update_gasregencmd_override(CAN_FIFOMailBox_TypeDef *override_msg) {
    gas_regen_override.RIR = override_msg->RIR;
    gas_regen_override.RDTR = override_msg->RDTR;
    gas_regen_override.RDLR = override_msg->RDLR;
    gas_regen_override.RDHR = override_msg->RDHR;

    gas_regen_override_ttl = 50;
}


bool handle_update_gasregencmd_override_rolling_counter(uint32_t rolling_counter) {
  gas_regen_override.RDLR &= 0xFFFFFF3FU;
  gas_regen_override.RDLR |= (rolling_counter << 6);
  uint32_t checksum3 = (0x100U - ((gas_regen_override.RDLR & 0xFF000000U) >> 24) - rolling_counter) & 0xFFU;

  gas_regen_override.RDHR &= 0x00FFFFFFU;
  gas_regen_override.RDHR = gas_regen_override.RDHR | (checksum3 << 24);

  // GAS/REGEN: safety check - TODO disable system instead of just dropping out of saftey range messages
  int gas_regen = ((GET_BYTE(&gas_regen_override, 2) & 0x7FU) << 5) + ((GET_BYTE(&gas_regen_override, 3) & 0xF8U) >> 3);
  if (gas_regen > GM_MAX_GAS) {
    return false;
  }
  return true;
}

void handle_update_acc_status_override(CAN_FIFOMailBox_TypeDef *override_msg) {
  acc_status_override.RIR = override_msg->RIR;
  acc_status_override.RDTR = override_msg->RDTR;
  acc_status_override.RDLR = override_msg->RDLR;
  acc_status_override.RDHR = override_msg->RDHR;

  acc_status_ttl = 50;
}

int fwd_filter(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  uint32_t addr = to_fwd->RIR>>21;
  
  // CAR to ASCM
  if (bus_num == 0) {
    return 2;
  }

  // OBJ
  if (bus_num == 1) {
    // 0x180 == lkasteeringcmd proxy
    if (addr == 0x180) {
      handle_update_steering_override(to_fwd);
    }
    // 0x2CB == gasregencmd proxy
    if (addr == 0x2CB) {
      handle_update_gasregencmd_override(to_fwd);
    }
    // 0x370 == ASCMActiveCruiseControlStatus proxy
    if (addr == 0x370) {
      handle_update_acc_status_override(to_fwd);
    }
    return -1;
  }

  // ASCM to CAR
  if (bus_num == 2) {
    // 0x180 == lkasteeringcmd : Only update, steering is more complex because it cycles between 10 Hz and 50 Hz
    // so this interceptor needs to be the heartbeat
    if (addr == 0x180) {
      handle_update_steering_oem(to_fwd);
      return -1;
    }
    // 0x2CB == gasregencmd
    if (addr == 0x2CB) {
      if (gas_regen_override_ttl > 0) {
	uint32_t curr_rolling_counter = (to_fwd->RDLR & 0xC0U) >> 6;
	if (handle_update_gasregencmd_override_rolling_counter(curr_rolling_counter)) {
          to_fwd->RIR = gas_regen_override.RIR;
          to_fwd->RDTR = gas_regen_override.RDTR;
          to_fwd->RDLR = gas_regen_override.RDLR;
          to_fwd->RDHR = gas_regen_override.RDHR;
	}
      }
      return 0;
    }
    // 0x370 == ASCMActiveCruiseControlStatus
    if (addr == 0x370) {
      ascm_acc_cmd_active = (to_fwd->RDLR >> 23) & 1U;
      if (acc_status_ttl > 0) {
        to_fwd->RIR = acc_status_override.RIR;
        to_fwd->RDTR = acc_status_override.RDTR;
        to_fwd->RDLR = acc_status_override.RDLR;
        to_fwd->RDHR = acc_status_override.RDHR;
      }
      return 0;
    }

    return 0;
  }

  return -1;
}

// CAN receive handlers
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
  while (CAN->RF0R & CAN_RF0R_FMP0) {
    if (can_number == 0) {
      can0_rx_cnt++;
    }
    if (can_number == 1) {
      can1_rx_cnt++;
    }
    if (can_number == 2) {
      can2_rx_cnt++;
    }

    // add to my fifo
    CAN_FIFOMailBox_TypeDef to_tx;
    to_tx.RIR = CAN->sFIFOMailBox[0].RIR | 1; //TXRQ
    to_tx.RDTR = CAN->sFIFOMailBox[0].RDTR;
    to_tx.RDLR = CAN->sFIFOMailBox[0].RDLR;
    to_tx.RDHR = CAN->sFIFOMailBox[0].RDHR;

    // modify RDTR for our API
    to_tx.RDTR = (to_tx.RDTR & 0xFFFF000F) | (bus_number << 4);

    // forwarding
    int bus_fwd_num = fwd_filter(bus_number, &to_tx);
    if (bus_fwd_num != -1) {
      can_push(can_queues[bus_fwd_num], &to_tx);
      process_can(CAN_NUM_FROM_BUS_NUM(bus_fwd_num));
    }

    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_TX_IRQHandler(void) { process_can(0); }
void CAN1_RX0_IRQHandler() { can_rx(0); }
void CAN1_SCE_IRQHandler() { can_sce(CAN1); }

void CAN2_TX_IRQHandler(void) { process_can(1); }
void CAN2_RX0_IRQHandler() { can_rx(1); }
void CAN2_SCE_IRQHandler() { can_sce(CAN2); }

void CAN3_TX_IRQHandler(void) { process_can(2); }
void CAN3_RX0_IRQHandler() { can_rx(2); }
void CAN3_SCE_IRQHandler() { can_sce(CAN3); }
