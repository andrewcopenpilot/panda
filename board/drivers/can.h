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


chassis: 0xC1: electronic brake control module
chassis: 0xC5: electronic brake control module
chassis: 0x130: multi axis accelerometer
chassis: 0x140: multi axis accelerometer
chassis: 0x170: electronic brake control module
chassis: 0x1E5: steering angle sensor
chassis: 0xC0: Seems to be needed for auto highbeams

powertrain proxy 0x2CA gas regen proxy
*/

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

const int GM_MAX_STEER = 300;
const int GM_MAX_RT_DELTA = 128;          // max delta torque allowed for real time checks
const uint32_t GM_RT_INTERVAL = 250000;    // 250ms between real time checks
const int GM_MAX_RATE_UP = 7;
const int GM_MAX_RATE_DOWN = 17;
const int GM_DRIVER_TORQUE_ALLOWANCE = 50;
const int GM_DRIVER_TORQUE_FACTOR = 4;
const int GM_MAX_GAS = 3072;
const int GM_MAX_REGEN = 1404;
const int GM_MAX_BRAKE = 350;

int can_err_cnt = 0;
int can0_mailbox_full_cnt = 0;
int can1_mailbox_full_cnt = 0;
int can0_rx_cnt = 0;
int can2_rx_cnt = 0;
int can0_tx_cnt = 0;
int can2_tx_cnt = 0;

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number);
bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem);

// overrides
CAN_FIFOMailBox_TypeDef steering_override;
volatile int steering_override_ttl = 0;
int steering_rolling_counter;

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
        if (can_number == 2) {
          can2_tx_cnt++;
        }
      }
    }

    exit_critical_section();
  }
}

void can_init(uint8_t can_number) {
  if (can_number == 0xff) return;

  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  set_can_enable(CAN, 1);
  can_set_speed(can_number);

  // accept all filter
  CAN->FMR |= CAN_FMR_FINIT;

  // no mask
  CAN->FS1R|=CAN_FS1R_FSC0;

  if (can_number == 99) {
    CAN->sFilterRegister[0].FR1 = 0x24B<<21;
    CAN->sFilterRegister[0].FR2 = 0xF1<<21;
    CAN->sFilterRegister[1].FR1 = 0xC9<<21;
    CAN->sFilterRegister[1].FR2 = 0x1E9<<21;
    CAN->sFilterRegister[2].FR1 = 0x1C4<<21;
    CAN->sFilterRegister[2].FR2 = 0x1C5<<21;
    CAN->sFilterRegister[3].FR1 = 0x1F5<<21;
    CAN->sFilterRegister[3].FR2 = 0x1E1<<21;
    CAN->sFilterRegister[4].FR1 = 0x214<<21;
    CAN->sFilterRegister[4].FR2 = 0x230<<21;
    CAN->sFilterRegister[5].FR1 = 0x34A<<21;
    CAN->sFilterRegister[5].FR2 = 0x12A<<21;
    CAN->sFilterRegister[6].FR1 = 0x135<<21;
    CAN->sFilterRegister[6].FR2 = 0x184<<21;
    CAN->sFilterRegister[7].FR1 = 0x1F1<<21;
    CAN->sFilterRegister[7].FR2 = 0x140<<21;
    CAN->sFilterRegister[8].FR1 = 0x2CA<<21;
    CAN->sFilterRegister[8].FR2 = 0x17F<<21;
    CAN->sFilterRegister[9].FR1 = 0x36F<<21;
    CAN->sFilterRegister[9].FR1 = 0x36F<<21;
    CAN->sFilterRegister[14].FR1 = 0;
    CAN->sFilterRegister[14].FR2 = 0;
    CAN->FM1R |= CAN_FM1R_FBM0 | CAN_FM1R_FBM1 | CAN_FM1R_FBM2 | CAN_FM1R_FBM3 | CAN_FM1R_FBM4 | CAN_FM1R_FBM5 | CAN_FM1R_FBM6 | CAN_FM1R_FBM7 | CAN_FM1R_FBM8 | CAN_FM1R_FBM9; 
    CAN->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 | CAN_FA1R_FACT2 | CAN_FA1R_FACT3 | CAN_FA1R_FACT4 | CAN_FA1R_FACT5 | CAN_FA1R_FACT6 | CAN_FA1R_FACT7 | CAN_FA1R_FACT8 | CAN_FA1R_FACT9;
    CAN->FA1R |= (1 << 14);
    CAN->FFA1R = 0x00000000;
  }
  else {
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->sFilterRegister[14].FR1 = 0;
    CAN->sFilterRegister[14].FR2 = 0;
    CAN->FA1R |= 1 | (1 << 14);
  }

  CAN->FMR &= ~(CAN_FMR_FINIT);

  // enable certain CAN interrupts
  CAN->IER |= CAN_IER_TMEIE | CAN_IER_FMPIE0;
  //NVIC_EnableIRQ(CAN_IER_TMEIE);
  switch (can_number) {
    case 0:
      NVIC_EnableIRQ(CAN1_TX_IRQn);
      NVIC_EnableIRQ(CAN1_RX0_IRQn);
      NVIC_EnableIRQ(CAN1_SCE_IRQn);
      break;
    case 1:
      NVIC_EnableIRQ(CAN2_TX_IRQn);
      NVIC_EnableIRQ(CAN2_RX0_IRQn);
      NVIC_EnableIRQ(CAN2_SCE_IRQn);
      break;
    case 2:
      NVIC_EnableIRQ(CAN3_TX_IRQn);
      NVIC_EnableIRQ(CAN3_RX0_IRQn);
      NVIC_EnableIRQ(CAN3_SCE_IRQn);
      break;
  }

  // in case there are queued up messages
  process_can(can_number);
}

void can_init_hw(uint8_t can_number) {
  if (can_number == 0xff) return;

  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  set_can_enable(CAN, 1);
  can_set_speed(can_number);

  // accept all filter
  CAN->FMR |= CAN_FMR_FINIT;

  // no mask
  CAN->FS1R|=CAN_FS1R_FSC0;

  if (can_number == 0) {
    CAN->sFilterRegister[0].FR1 = 0x24B<<21;
    CAN->sFilterRegister[0].FR2 = 0xF1<<21;
    CAN->sFilterRegister[1].FR1 = 0xC9<<21;
    CAN->sFilterRegister[1].FR2 = 0x1E9<<21;
    CAN->sFilterRegister[2].FR1 = 0x1C4<<21;
    CAN->sFilterRegister[2].FR2 = 0x1C5<<21;
    CAN->sFilterRegister[3].FR1 = 0x1F5<<21;
    CAN->sFilterRegister[3].FR2 = 0x1E1<<21;
    CAN->sFilterRegister[4].FR1 = 0x214<<21;
    CAN->sFilterRegister[4].FR2 = 0x230<<21;
    CAN->sFilterRegister[5].FR1 = 0x34A<<21;
    CAN->sFilterRegister[5].FR2 = 0x12A<<21;
    CAN->sFilterRegister[6].FR1 = 0x135<<21;
    CAN->sFilterRegister[6].FR2 = 0x184<<21;
    CAN->sFilterRegister[7].FR1 = 0x1F1<<21;
    CAN->sFilterRegister[7].FR2 = 0x140<<21;
    CAN->sFilterRegister[8].FR1 = 0x2CA<<21;
    CAN->sFilterRegister[8].FR2 = 0x17F<<21;
    CAN->sFilterRegister[9].FR1 = 0x36F<<21;
    CAN->sFilterRegister[9].FR1 = 0x36F<<21;
    CAN->sFilterRegister[14].FR1 = 0;
    CAN->sFilterRegister[14].FR2 = 0;
    CAN->FM1R |= CAN_FM1R_FBM0 | CAN_FM1R_FBM1 | CAN_FM1R_FBM2 | CAN_FM1R_FBM3 | CAN_FM1R_FBM4 | CAN_FM1R_FBM5 | CAN_FM1R_FBM6 | CAN_FM1R_FBM7 | CAN_FM1R_FBM8 | CAN_FM1R_FBM9; 
    CAN->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 | CAN_FA1R_FACT2 | CAN_FA1R_FACT3 | CAN_FA1R_FACT4 | CAN_FA1R_FACT5 | CAN_FA1R_FACT6 | CAN_FA1R_FACT7 | CAN_FA1R_FACT8 | CAN_FA1R_FACT9;
    CAN->FA1R |= (1 << 14);
    CAN->FFA1R = 0x00000000;
  }
  else {
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->sFilterRegister[14].FR1 = 0;
    CAN->sFilterRegister[14].FR2 = 0;
    CAN->FA1R |= 1 | (1 << 14);
  }

  CAN->FMR &= ~(CAN_FMR_FINIT);

  // enable certain CAN interrupts
  CAN->IER |= CAN_IER_TMEIE | CAN_IER_FMPIE0;
  //NVIC_EnableIRQ(CAN_IER_TMEIE);
  switch (can_number) {
    case 0:
      NVIC_EnableIRQ(CAN1_TX_IRQn);
      NVIC_EnableIRQ(CAN1_RX0_IRQn);
      NVIC_EnableIRQ(CAN1_SCE_IRQn);
      break;
    case 1:
      NVIC_EnableIRQ(CAN2_TX_IRQn);
      NVIC_EnableIRQ(CAN2_RX0_IRQn);
      NVIC_EnableIRQ(CAN2_SCE_IRQn);
      break;
    case 2:
      NVIC_EnableIRQ(CAN3_TX_IRQn);
      NVIC_EnableIRQ(CAN3_RX0_IRQn);
      NVIC_EnableIRQ(CAN3_SCE_IRQn);
      break;
  }

  // in case there are queued up messages
  process_can(can_number);
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


void can_init_all() {
  for (int i=0; i < CAN_MAX; i++) {
    can_init(i);
  }
}

// single wire (low speed) GMLAN only used for button presses to change mode in the future
void can_set_gmlan(int bus) {
  if (bus == -1 || bus != can_num_lookup[3]) {
    // GMLAN OFF
    switch (can_num_lookup[3]) {
      case 1:
        puts("disable GMLAN on CAN2\n");
        set_can_mode(1, 0);
        bus_lookup[1] = 1;
        can_num_lookup[1] = 1;
        can_num_lookup[3] = -1;
        can_init(1);
        break;
      case 2:
        puts("disable GMLAN on CAN3\n");
        set_can_mode(2, 0);
        bus_lookup[2] = 2;
        can_num_lookup[2] = 2;
        can_num_lookup[3] = -1;
        can_init(2);
        break;
    }
  }

  if (bus == 1) {
    puts("GMLAN on CAN2\n");
    // GMLAN on CAN2
    set_can_mode(1, 1);
    bus_lookup[1] = 3;
    can_num_lookup[1] = -1;
    can_num_lookup[3] = 1;
    can_init(1);
  } else if (bus == 2 && revision == PANDA_REV_C) {
    puts("GMLAN on CAN3\n");
    // GMLAN on CAN3
    set_can_mode(2, 1);
    bus_lookup[2] = 3;
    can_num_lookup[2] = -1;
    can_num_lookup[3] = 2;
    can_init(2);
  }
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
  TIM3->ARR = 1000;	          // Auto reload value 1000
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
  }
}

// ***************************** CAN *****************************

void handle_update_steering_override(CAN_FIFOMailBox_TypeDef *override_msg) {
    steering_override.RIR = (384 << 21) | (override_msg->RIR & 0x1FFFFFU);
    steering_override.RDTR = override_msg->RDTR;
    steering_override.RDLR = override_msg->RDLR;
    steering_override.RDHR = override_msg->RDHR;

    steering_override_ttl = 2;
}

void handle_update_gasregencmd_override(CAN_FIFOMailBox_TypeDef *override_msg) {
    gas_regen_override.RIR = (715 << 21) | (override_msg->RIR & 0x1FFFFFU);
    gas_regen_override.RDTR = override_msg->RDTR;
    gas_regen_override.RDLR = override_msg->RDLR;
    gas_regen_override.RDHR = override_msg->RDHR;

    gas_regen_override_ttl = 2;
}


bool handle_update_gasregencmd_override_rolling_counter(uint32_t rolling_counter) {
  gas_regen_override.RDLR &= 0xFFFFFF3FU;
  gas_regen_override.RDLR |= (rolling_counter << 6);
  uint32_t checksum3 = (0x100U - ((gas_regen_override.RDLR & 0xFF000000U) >> 24) - rolling_counter) & 0xFFU;

  gas_regen_override.RDHR &= 0x00FFFFFFU;
  gas_regen_override.RDHR = gas_regen_override.RDHR | (checksum3 << 24);

  // GAS/REGEN: safety check - TODO disable system instead of just dropping out of saftey range messages
  //int gas_regen = ((GET_BYTE(&gas_regen_override, 2) & 0x7FU) << 5) + ((GET_BYTE(&gas_regen_override, 3) & 0xF8U) >> 3);
  //bool apply = GET_BYTE(&gas_regen_override, 0) & 1U;
  //if (apply || (gas_regen != GM_MAX_REGEN)) {
  //  return false;
  //}
  //if (gas_regen > GM_MAX_GAS) {
  //  return false;
  //}
  return true;
}

void handle_update_acc_status_override(CAN_FIFOMailBox_TypeDef *override_msg) {
  acc_status_override.RIR = (880 << 21) | (override_msg->RIR & 0x1FFFFFU);
  acc_status_override.RDTR = override_msg->RDTR;
  acc_status_override.RDLR = override_msg->RDLR;
  acc_status_override.RDHR = override_msg->RDHR;

  acc_status_ttl = 2;
}

int fwd_filter(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  uint32_t addr = to_fwd->RIR>>21;
  
  // CAR to ASCM
  if (bus_num == 0) {
    // 383 == lkasteeringcmd proxy
    //if (addr == 383) {
    //  handle_update_steering_override(to_fwd);
    //  to_fwd->RIR = steering_override.RIR;
    //  to_fwd->RDTR = steering_override.RDTR;
    //  to_fwd->RDLR = steering_override.RDLR;
    //  to_fwd->RDHR = steering_override.RDHR;
    //  return 0;
    //}
    // 714 == gasregencmd proxy
    if (addr == 714) {
      handle_update_gasregencmd_override(to_fwd);
      return -1;
    }
    // 879 == ASCMActiveCruiseControlStatus proxy
    if (addr == 879) {
      handle_update_acc_status_override(to_fwd);
    }
    return 2;
  }

  // ASCM to CAR
  if (bus_num == 2) {
    // 384 == lkasteeringcmd
    if (addr == 384) {
      return -1;
      //if (steering_override_ttl > 0) {
      //  to_fwd->RIR = steering_override.RIR;
      //  to_fwd->RDTR = steering_override.RDTR;
      //  to_fwd->RDLR = steering_override.RDLR;
      //  to_fwd->RDHR = steering_override.RDHR; 
      //}
      //else {
	//puts("missed steering\n");
      //  return -1;
      //}
      //return 0;
    }
    // 715 == gasregencmd
    if (addr == 715) {
      if (gas_regen_override_ttl > 0) {
	uint32_t curr_rolling_counter = (to_fwd->RDLR & 0xC0U) >> 6;
	if (handle_update_gasregencmd_override_rolling_counter(curr_rolling_counter)) {
          to_fwd->RIR = gas_regen_override.RIR;
          to_fwd->RDTR = gas_regen_override.RDTR;
          to_fwd->RDLR = gas_regen_override.RDLR;
          to_fwd->RDHR = gas_regen_override.RDHR;
	}
      }
      //else {
      //  return -1;
      //}
      return 0;
    }
    // 880 == ASCMActiveCruiseControlStatus
    if (addr == 880) {
      if (acc_status_ttl > 0) {
        to_fwd->RIR = acc_status_override.RIR;
        to_fwd->RDTR = acc_status_override.RDTR;
        to_fwd->RDLR = acc_status_override.RDLR;
        to_fwd->RDHR = acc_status_override.RDHR;
      }
      //else {
      //  return -1;
      //}
      return 0;
    }

    return 0;
  }

  return -1;
}

// CAN receive handlers
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  //enter_critical_section();
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
  while (CAN->RF0R & CAN_RF0R_FMP0) {
    if (can_number == 0) {
      can0_rx_cnt++;
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
      //can_tx(bus_fwd_num, &to_tx);
      can_push(can_queues[bus_fwd_num], &to_tx);
      process_can(CAN_NUM_FROM_BUS_NUM(bus_fwd_num));
    }

    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
  //exit_critical_section();
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
