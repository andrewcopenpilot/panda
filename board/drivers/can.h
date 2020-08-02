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

const int GM_MAX_BRAKE = 350;

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

//overrides
CAN_FIFOMailBox_TypeDef brake_override;
volatile int brake_override_ttl = 0;
int brake_rolling_counter;

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

  // clear current send
  CAN->TSR |= CAN_TSR_ABRQ0;
  CAN->MSR &= ~(CAN_MSR_ERRI);
  CAN->MSR = CAN->MSR;

  exit_critical_section();
}

void ttl_timer_init() {
  TIM3->PSC = 4800-1;           // Set prescaler to 24 000 (PSC + 1)
  TIM3->ARR = 1000;               // Auto reload value 1000
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
    exit_critical_section();
    
    tick++;
    if (tick % 1 == 0) {
      send_interceptor_status();
    }

  }
}

void send_interceptor_status() {
    CAN_FIFOMailBox_TypeDef status;

    status.RIR = (0x376 << 21) | 1;
    status.RDTR = 2;
    status.RDLR = 0x01020304;
    status.RDHR = 0x05060708;

    can_push(can_queues[1], &status);
    process_can(CAN_NUM_FROM_BUS_NUM(1));
}

void handle_update_brake_override(CAN_FIFOMailBox_TypeDef *override_msg) {
  brake_override.RIR = override_msg->RIR
  brake_override.RDTR = override_msg->RDTR;
  brake_override.RDLR = override_msg->RDLR;
  brake_override.RDHR = override_msg->RDHR;

  brake_override_ttl = 5;
}

bool handle_update_brake_override_rolling_counter(uint32_t rolling_counter) {
  uint32_t data = brake_override.RDLR & 0xFFFFU;
  uint32_t dataswap = (data >> 8) | ((data << 8) & 0xFF00U);
  uint32_t checksum = (0x10000U - dataswap - rolling_counter) & 0xFFFFU;
  uint32_t checksumswap = (checksum >> 8) | ((checksum << 8) & 0xFF00U);
  brake_override.RDLR &= 0x0000FFFFU;
  brake_override.RDLR = brake_override.RDLR | (checksumswap << 16);
  brake_override.RDHR &= ~0x3U;
  brake_override.RDHR |= (rolling_counter & 0x3U);

  // BRAKE: safety check
  int brake = ((GET_BYTE(&brake_override, 0) & 0xFU) << 8) + GET_BYTE(&brake_override, 1);
  brake = (0x1000 - brake) & 0xFFF;
  if (brake > GM_MAX_BRAKE) {
    return false;
  }
  return true;
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
    return -1;
  }

  // ASCM to CAR
  if (bus_num == 2) {
    // brake messages
    if (addr == 0x315) {
	if (brake_override_ttl > 0) {
          uint32_t curr_rolling_counter = (to_fwd->RDHR & 0x3U);
	  if (handle_update_brake_override_rolling_counter(curr_rolling_counter)) {
	    to_fwd->RIR = brake_override.RIR;
            to_fwd->RDTR = brake_override.RDTR;
            to_fwd->RDLR = brake_override.RDLR;
            to_fwd->RDHR = brake_override.RDHR;
	  }
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
