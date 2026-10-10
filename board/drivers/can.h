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


chassis: 0xc1: electronic brake control module
chassis: 0xc5: electronic brake control module
chassis: 0x130: multi axis accelerometer
chassis: 0x140: multi axis accelerometer
chassis: 0x170: electronic brake control module
chassis: 0x1e5: steering angle sensor
chassis: 0xc0: seems to be needed for auto highbeams
*/

#define GET_ADDR(msg) ((((msg)->RIR & 4) != 0) ? ((msg)->RIR >> 3) : ((msg)->RIR >> 21))
#define GET_BYTE(msg, b) (((int)(b) > 3) ? (((msg)->RDHR >> (8U * ((unsigned int)(b) % 4U))) & 0XFFU) : (((msg)->RDLR >> (8U * (unsigned int)(b))) & 0xFFU))
#define CAN_TRANSMIT 1U

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

// Signed 12-bit EBCM acceleration request, in CAN counts.
const int GM_MIN_ACCEL_REQUEST = -400;
const int GM_MAX_ACCEL_REQUEST = 200;

bool brake_max_exceeded = false;

int can_err_cnt = 0;
int can0_mailbox_full_cnt = 0;
int can1_mailbox_full_cnt = 0;
int can0_rx_cnt = 0;
int can1_rx_cnt = 0;
int can2_rx_cnt = 0;
int can0_tx_cnt = 0;
int can1_tx_cnt = 0;
int can2_tx_cnt = 0;

uint32_t tick = 0;

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number);
bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem);
void send_interceptor_status();
void update_chassis_mode(void);
void fill_synth_315(CAN_FIFOMailBox_TypeDef *frame);
void rewrite_315_counter(CAN_FIFOMailBox_TypeDef *frame);

// Chassis operating mode. Mirrors the PT interceptor's interceptor_mode via
// the 0x375 broadcast on OBJ, with local overrides for safety:
//   M1 (ASCM PT)  — pass ASCM 0x315 through
//   M2 (OP PT)    — substitute with comma's 0x315 override
//   M3 (NO PT)    — failsafe: synthetic zero-brake on bus 0
// Forced to M3 if PT goes silent (pt_status_ttl expires) or if a local fault
// is latched. This keeps the chassis safe even if the PT broadcast is lost.
typedef enum {
  MODE_M1_ASCM_PT = 0,
  MODE_M2_OP_PT = 1,
  MODE_M3_NO_PT = 2,
} chassis_mode_t;

// Last-reported PT mode (decoded from incoming 0x375). Goes stale if PT
// broadcast stops; pt_status_ttl tracks freshness.
volatile chassis_mode_t pt_mode = MODE_M1_ASCM_PT;
volatile int pt_status_ttl = 0;
// Effective chassis mode, derived from pt_mode + local fault state.
// Set by update_chassis_mode() in the TIM3 IRQ.
volatile chassis_mode_t chassis_mode = MODE_M1_ASCM_PT;

//overrides
CAN_FIFOMailBox_TypeDef brake_override;
volatile int brake_override_ttl = 0;

// Interceptor-owned rolling counter for 0x315. Applied to every outbound
// frame on bus 0 regardless of mode, so the counter sequence the EBCM sees
// stays monotonic across M1/M2/M3 transitions.
volatile uint8_t synth_315_counter = 0;

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

        // clear interrupt. TSR is full of W1C bits — write the mask directly
        // rather than |= so we don't accidentally clear other completion flags
        // (RQCP1/2, ALST*, TERR*, ...) if they're set.
        CAN->TSR = CAN_TSR_RQCP0;
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
  CAN1->FM1R |= CAN_FM1R_FBM0 | CAN_FM1R_FBM1 | CAN_FM1R_FBM2 | CAN_FM1R_FBM3 | CAN_FM1R_FBM14 | CAN_FM1R_FBM15;

  // filter scale register - Set all filter banks to be 32-bit (vs dual 16 bit)
  CAN1->FS1R = 0x00000000; // Reset value

  // filter FIFO assignment register - Set all filters to store in FIFO 0
  CAN1->FFA1R = 0x00000000;

  // filter activation register - CAN_FA1R_FACTX bit activate the associated filter bank
  CAN1->FA1R = 0x00000000; // Reset value

  CAN1->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 | CAN_FA1R_FACT2 | CAN_FA1R_FACT3 | CAN_FA1R_FACT14 | CAN_FA1R_FACT15;

  //Set CAN 1 Filters CAR Chas
  CAN1->sFilterRegister[0].FR1 = 0xC1<<21;
  CAN1->sFilterRegister[0].FR2 = 0xC5<<21;
  CAN1->sFilterRegister[1].FR1 = 0x130<<21;
  CAN1->sFilterRegister[1].FR2 = 0x140<<21;
  CAN1->sFilterRegister[2].FR1 = 0x170<<21;
  CAN1->sFilterRegister[2].FR2 = 0x1E5<<21;
  CAN1->sFilterRegister[3].FR1 = 0xC0<<21;
  CAN1->sFilterRegister[3].FR2 = 0xC0<<21;

  // Set Can 2 Filters Obj
  CAN1->sFilterRegister[14].FR1 = 0x315<<21; // Friction brake proxy
  CAN1->sFilterRegister[14].FR2 = 0x375<<21; // PT Interceptor status
  CAN1->sFilterRegister[15].FR1 = 0x377<<21; // SW GMLAN proxy status
  CAN1->sFilterRegister[15].FR2 = 0x377<<21;

  CAN1->FMR &= ~(CAN_FMR_FINIT);

  // filter master register - Set filter init mode
  CAN3->FMR |= CAN_FMR_FINIT;

  // filter mode register - Set all filter banks to mask mode
  CAN3->FM1R = 0x00000000; // Reset value

  // filter scale register - Set all filter banks to be dual 16-bit (vs 32 bit)
  CAN1->FS1R = 0x00000000; // Reset value (all 16 bit mode)
  CAN1->FS1R = CAN_FS1R_FSC; // Set all filter banks to 32 bit mode

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

  // abort the in-flight send on mailbox 0. ABRQ0 is set-by-software /
  // reset-by-hardware (reads as 0), so a direct write sets it without
  // disturbing TSR's W1C status bits (RQCP*/TXOK*/ALST*/TERR*).
  CAN->TSR = CAN_TSR_ABRQ0;
  // MSR = MSR clears all set W1C flags (ERRI/WKUI/SLAKI): the read returns
  // their current values, and writing 1 to a W1C bit clears it.
  CAN->MSR = CAN->MSR;

  exit_critical_section();
}

// Decode the PT interceptor's 0x375 broadcast and refresh pt_status_ttl.
// Mode is at byte 0 bits 1-2. Value 3 isn't a valid mode (only M1/M2/M3 are
// encoded) — clamp anything unexpected to M3 so a corrupted frame doesn't
// accidentally release us from failsafe.
void handle_update_pt_status(CAN_FIFOMailBox_TypeDef *msg) {
  uint8_t byte0 = GET_BYTE(msg, 0);
  uint8_t mode_bits = (byte0 >> 1) & 0x3U;
  if (mode_bits <= (uint8_t)MODE_M3_NO_PT) {
    pt_mode = (chassis_mode_t)mode_bits;
  } else {
    pt_mode = MODE_M3_NO_PT;
  }
  pt_status_ttl = 5;  // ~500 ms grace at TIM3_HZ=10
}

// Effective chassis mode = pt_mode unless we have a reason to be in failsafe:
//   - PT broadcast has gone silent (pt_status_ttl decayed to 0)
//   - Local brake-bound safety flag has latched
// Runs in TIM3 IRQ. Structuring as a pure derivation lets this collapse
// cleanly when chassis + PT eventually merge into a single 5-CAN MCU.
void update_chassis_mode(void) {
  if ((pt_status_ttl == 0) || brake_max_exceeded) {
    chassis_mode = MODE_M3_NO_PT;
  } else {
    chassis_mode = pt_mode;
  }
}

// TIM3 ticks at 10 Hz. APB1 timer clock is 48 MHz on this part, so:
//   48 MHz / (PSC+1=4800) = 10 kHz counter clock
//   10 kHz / (ARR+1=1000) = 10 Hz update events
// Downstream consumers of this rate: status broadcast on 0x376, and the
// brake_override_ttl decrement.
#define TIM3_HZ 10
void ttl_timer_init() {
  TIM3->PSC = 4800-1;        // counter clock = 48 MHz / 4800 = 10 kHz
  TIM3->ARR = 1000-1;        // update event   = 10 kHz / 1000 = 10 Hz
  TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
  TIM3->CR1 = TIM_CR1_CEN;   // Enable timer

  NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void) {
  if (TIM3->SR & TIM_SR_UIF) { // if UIF flag is set
    TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
    enter_critical_section();
    if (brake_override_ttl > 0) {
      brake_override_ttl--;
    }
    if (pt_status_ttl > 0) {
      pt_status_ttl--;
    }
    exit_critical_section();

    update_chassis_mode();

    tick++;
    send_interceptor_status();
  }
}

// 0x376 status frame to OBJ. PT interceptor and comma both read this.
//   byte 0 bit 0  : SystemHealthy (1 = no latched local faults)
//   byte 0 bit 1  : BrakeExceeded (sticky)
//   byte 0 bits 2-3: ChassisMode (0=M1, 1=M2, 2=M3)
//   all other bytes: reserved (0)
void send_interceptor_status() {
    CAN_FIFOMailBox_TypeDef status;

    // Healthy = no latched faults. Currently brake_max_exceeded is the only
    // latched fault; AND in any future fault flags here.
    bool healthy = !brake_max_exceeded;

    union {
        uint8_t bytes[8];
        struct { uint32_t lo; uint32_t hi; };
    } data = {{0}};

    const uint8_t SYSTEM_HEALTHY_BIT  = 0;
    const uint8_t BRAKE_EXCEEDED_BIT  = 1;
    const uint8_t MODE_SHIFT          = 2;  // 2 bits wide
    data.bytes[0] = ((healthy ? 1U : 0U)               << SYSTEM_HEALTHY_BIT)
                  | ((brake_max_exceeded ? 1U : 0U)    << BRAKE_EXCEEDED_BIT)
                  | (((uint8_t)chassis_mode & 0x3U)    << MODE_SHIFT);

    status.RIR = (0x376 << 21) | 1;
    status.RDTR = 8;
    status.RDLR = data.lo;
    status.RDHR = data.hi;

    can_push(can_queues[1], &status);
    process_can(CAN_NUM_FROM_BUS_NUM(1));
}

void handle_update_brake_override(CAN_FIFOMailBox_TypeDef *override_msg) {
  brake_override.RIR = override_msg->RIR;
  brake_override.RDTR = override_msg->RDTR;
  brake_override.RDLR = override_msg->RDLR;
  brake_override.RDHR = override_msg->RDHR;

  brake_override_ttl = 5;
}

// Apply interceptor-owned rolling counter to an in-flight 0x315 frame and
// recompute the 16-bit checksum. Counter lives at byte 4 bits 0-1; checksum
// covers (mode << 12 | brake) which is bytes 0-1 (big-endian).
void rewrite_315_counter(CAN_FIFOMailBox_TypeDef *frame) {
  uint32_t data = frame->RDLR & 0xFFFFU;
  uint32_t dataswap = (data >> 8) | ((data << 8) & 0xFF00U);
  uint32_t checksum = (0x10000U - dataswap - synth_315_counter) & 0xFFFFU;
  uint32_t checksumswap = (checksum >> 8) | ((checksum << 8) & 0xFF00U);

  // Place checksum into bytes 2-3
  frame->RDLR = (frame->RDLR & 0x0000FFFFU) | (checksumswap << 16);
  // Place counter into byte 4 bits 0-1
  frame->RDHR = (frame->RDHR & ~0x3U) | (synth_315_counter & 0x3U);

  synth_315_counter = (synth_315_counter + 1U) & 0x3U;
}

// Fill a frame with the synthetic 0x315 failsafe payload: FrictionBrakeMode=0x0
// (ACC not active), apply_brake=0. We deliberately don't claim "ACC active, no
// brake" (mode=0x1) — failsafe should tell the EBCM that nothing is in control,
// not that ACC is alive but idle. Counter and checksum are set by
// rewrite_315_counter() afterward.
void fill_synth_315(CAN_FIFOMailBox_TypeDef *frame) {
  frame->RIR = (0x315 << 21) | 1U;
  frame->RDTR = 5U;  // DLC = 5
  // byte 0 = (mode << 4) | (brake_enc bits 8-11) = (0x0 << 4) | 0x0 = 0x00
  // byte 1 = brake_enc bits 0-7 = 0x00
  // bytes 2-3 (checksum) set by rewrite_315_counter
  frame->RDLR = 0x00000000U;
  // byte 4 (counter) set by rewrite_315_counter; bytes 5-7 unused (DLC=5)
  frame->RDHR = 0x00000000U;
}

// ***************************** CAN *****************************

int fwd_filter(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  uint32_t addr = to_fwd->RIR>>21;

  // CAR to ASCM
  if (bus_num == 0) {
    return 2;
  }

  // Obj
  if (bus_num == 1) {
    // brake proxy
    if (addr == 0x315) {
      handle_update_brake_override(to_fwd);
    }
    // PT interceptor status — drives chassis mode
    if (addr == 0x375) {
      handle_update_pt_status(to_fwd);
    }
    return -1;
  }

  // ASCM to CAR
  if (bus_num == 2) {
    // brake messages
    if (addr == 0x315) {
      // M3 failsafe: emit synthetic "no brake commanded" in place of ASCM frame.
      // M2 (OP PT): substitute payload with OP's override, subject to safety bound;
      //             out-of-range requests latch a fault and fall back to synthetic.
      // M1 (ASCM PT): pass ASCM payload through unchanged.
      if (chassis_mode == MODE_M3_NO_PT) {
        fill_synth_315(to_fwd);
      } else if (chassis_mode == MODE_M2_OP_PT && brake_override_ttl > 0) {
        int request = ((GET_BYTE(&brake_override, 0) & 0xFU) << 8)
                    + GET_BYTE(&brake_override, 1);
        // Decode the signed 12-bit acceleration request before checking bounds.
        if (request & 0x800) {
          request -= 0x1000;
        }
        if (request < GM_MIN_ACCEL_REQUEST || request > GM_MAX_ACCEL_REQUEST) {
          brake_max_exceeded = true;
          fill_synth_315(to_fwd);
        } else {
          to_fwd->RIR = brake_override.RIR;
          to_fwd->RDTR = brake_override.RDTR;
          to_fwd->RDLR = brake_override.RDLR;
          to_fwd->RDHR = brake_override.RDHR;
        }
      }
      // Apply interceptor-owned counter on every outbound frame so the sequence
      // stays monotonic across M1/M2/M3 transitions.
      rewrite_315_counter(to_fwd);
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
      //can_tx(bus_fwd_num, &to_tx);
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
