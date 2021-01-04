#define GET_ADDR(msg) ((((msg)->RIR & 4) != 0) ? ((msg)->RIR >> 3) : ((msg)->RIR >> 21))

// IRQs: CAN1_TX, CAN1_RX0, CAN1_SCE, CAN2_TX, CAN2_RX0, CAN2_SCE, CAN3_TX, CAN3_RX0, CAN3_SCE

#define CAN_MAX 3

#define can_buffer(x, size) \
  CAN_FIFOMailBox_TypeDef elems_##x[size]; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = size, .elems = (CAN_FIFOMailBox_TypeDef *)&elems_##x };

#define CAN_TRANSMIT 1
#define CAN_EXTENDED 4

can_buffer(tx1_q, 0x100)
can_buffer(tx2_q, 0x100)
can_buffer(tx3_q, 0x100)
can_ring *can_queues[] = {&can_tx1_q, &can_tx2_q, &can_tx3_q};

int can_err_cnt = 0;
int can0_rx_cnt = 0;
int can1_rx_cnt = 0;
int can2_rx_cnt = 0;
int can0_tx_cnt = 0;
int can1_tx_cnt = 0;
int can2_tx_cnt = 0;

uint32_t tick = 0;

bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem);
void send_interceptor_status();

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
uint32_t can_speed[] = {333, 5000, 333, 333};

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
    //CAN->MCR = CAN_MCR_TTCM | CAN_MCR_ABOM | CAN_MCR_NART;
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
	if ( bus_number == 2 && GET_ADDR(&to_send) == 0x100) {
	  puts("\nsetting hv wakeup\n");
	  set_sw_gmlan_hv_wakeup();
	  delay(43668); // Sleep for about 40ms
	}

        // only send if we have received a packet
        CAN->sTxMailBox[0].TDLR = to_send.RDLR;
        CAN->sTxMailBox[0].TDHR = to_send.RDHR;
        CAN->sTxMailBox[0].TDTR = to_send.RDTR;
        CAN->sTxMailBox[0].TIR = to_send.RIR;

	if ( bus_number == 2 && GET_ADDR(&to_send) == 0x100) {
	  delay(45459); // Sleep for about 5ms
	  puts("\nsetting normal mode\n");
	  set_sw_gmlan_normal();
	  delay(43668); // Sleep for about 40ms
	}

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
  CAN1->FM1R |= CAN_FM1R_FBM14 | CAN_FA1R_FACT15;

  // filter scale register - Set all filter banks to be dual 16-bit (vs 32 bit)
  CAN1->FS1R = 0x00000000; // Reset value (all 16 bit mode)
  CAN1->FS1R = CAN_FS1R_FSC; // Set all filter banks to 32 bit mode

  // filter FIFO assignment register - Set all filters to store in FIFO 0
  CAN1->FFA1R = 0x00000000;

  // filter activation register - CAN_FA1R_FACTX bit activate the associated filter bank
  CAN1->FA1R = 0x00000000; // Reset value
  CAN1->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT14 | CAN_FA1R_FACT15;

  //Set CAN 1 Filters SW GMLAN
  //CAN1->sFilterRegister[0].FR1 = (0x10306099<<3) | CAN_EXTENDED;
  //CAN1->sFilterRegister[0].FR2 = (0x10306099<<3) | CAN_EXTENDED;
  CAN1->sFilterRegister[0].FR1 = 0;
  CAN1->sFilterRegister[0].FR2 = 0;




  // Set Can 2 Filters Object
  CAN1->sFilterRegister[14].FR1 = (0x104c006c<<3) | CAN_EXTENDED; // Openpilot lka icon
  CAN1->sFilterRegister[14].FR2 = 885 << 21; // PT Interceptor status
  CAN1->sFilterRegister[15].FR1 = 886 << 21; // Chas Interceptor status
  CAN1->sFilterRegister[15].FR2 = 886 << 21;

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


  can_set_speed(0);
  can_set_speed(1);
  can_set_speed(2);

  // in case there are queued up messages
  process_can(0);
  process_can(1);
  process_can(2);
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
    if (tick % 5 == 0) {
        send_interceptor_status();
    }
    tick++;
  }
}

// ***************************** CAN *****************************

int fwd_filter(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int addr = GET_ADDR(to_fwd);

  // SW GMLAN
  if (bus_num == 0) {
    if (can2_rx_cnt == 0) {
      return -1;
    }
    //puts("addr: ");
    //puth(addr);
    //puts(" end\n");
// if (addr == 0x00000620 ) {
//  puts("stay awake: RIR:");
//  puth(to_fwd->RIR);
//  puts(" RDTR: ");
//  puth(to_fwd->RDTR);
//  puts(" RDLR: ");
//  puth(to_fwd->RDLR);
//  puts(" RDHR: ");
//  puth(to_fwd->RDHR);
//  puts("\n");
// }


// if (addr == 0x100 ) { return 2;} // HV wakeup
 if (addr == 0x13ffe060 ) { return 2;} // Instrument Cluster
 if (addr == 0x13ffe0bb ) { return 2;} // Parking Assist Control Module
 if (addr == 0x108e0080 ) { return 2;} // Radio
 if (addr == 0x13ffe080 ) { return 2;} // Radio 
 if (addr == 0x10324058 ) { return 2;} // Inflatable Restraint Sensing and Diagnostics Module
 if (addr == 0x13ffe058 ) { return 2;} // Inflatable Restraint Sensing and Diagnostics Module
 if (addr == 0x10264040 ) { return 2;} // Body Control Module
 if (addr == 0x13ffe040 ) { return 2;} // Body Control Module
 if (addr == 0x10ec4040 ) { return 2;} // Body Control Module
 if (addr == 0x10ec8040 ) { return 2;} // Body Control Module
 if (addr == 0x10788040 ) { return 2;} // Body Control Module
 if (addr == 0x106c0040 ) { return 2;} // Body Control Module
 if (addr == 0x10248040 ) { return 2;} // Body Control Module
 if (addr == 0x10240040 ) { return 2;} // Body Control Module
 if (addr == 0x10210040 ) { return 2;} // Body Control Module
 if (addr == 0x00000620 ) { return 2;} // Bus off failure without this message, don't know which module sends it

 return -1;
 //return 2;


 //if (addr == 0x13ffe059 ) { return -1;} // Inflatable Restraint Sensing and Diagnostics Module (maybe not?)
/*
 if (addr == 0x102c6060 ) { return 2;}
 if (addr == 0x00000621 ) { return 2;}
 if (addr == 0x00000624 ) { return 2;}
 if (addr == 0x0000062c ) { return 2;}
 if (addr == 0x0c28c040 ) { return 2;}
 if (addr == 0x0c2f6040 ) { return 2;}
 if (addr == 0x0c2f8040 ) { return 2;}
 if (addr == 0x0c2fa040 ) { return 2;}
 if (addr == 0x0c2fe040 ) { return 2;}
 if (addr == 0x0c394040 ) { return 2;}
 if (addr == 0x0c6080af ) { return 2;}
 if (addr == 0x0c630040 ) { return 2;}
 if (addr == 0x0c6aa040 ) { return 2;}
 if (addr == 0x0c6b4040 ) { return 2;}
 if (addr == 0x0c714040 ) { return 2;}
 if (addr == 0x0c7920af ) { return 2;}
*/

 //if (addr == 0x0c800040 ) { return 2;}
 //if (addr == 0x1020c040 ) { return 2;}
 //if (addr == 0x10220040 ) { return 2;}
 //if (addr == 0x102240cb ) { return 2;}
 //if (addr == 0x1022a040 ) { return 2;}
 //if (addr == 0x1022c040 ) { return 2;}
 //if (addr == 0x1022e040 ) { return 2;}
 //if (addr == 0x10230040 ) { return 2;}
 
/*
 if (addr == 0x10242040 ) { return 2;}
 if (addr == 0x10244060 ) { return 2;}
 if (addr == 0x10250040 ) { return 2;}
 // Contains BODY control module
 //end contains body control module
 
 if (addr == 0x102680af ) { return 2;}
 if (addr == 0x1026c0cb ) { return 2;}
 if (addr == 0x102700cb ) { return 2;}
 if (addr == 0x102740cb ) { return 2;}
 if (addr == 0x1027a0cb ) { return 2;}
 */


/* 
 if (addr == 0x102820cb ) { return 2;}
 if (addr == 0x1029e097 ) { return 2;}
 if (addr == 0x102a8097 ) { return 2;}
 if (addr == 0x102aa097 ) { return 2;}
 if (addr == 0x102ac097 ) { return 2;}
 if (addr == 0x102c0040 ) { return 2;}
 if (addr == 0x102c2080 ) { return 2;}
 if (addr == 0x102c40cb ) { return 2;}
 if (addr == 0x102ca040 ) { return 2;}
 if (addr == 0x102cc040 ) { return 2;}
 if (addr == 0x102ce040 ) { return 2;}
 if (addr == 0x102d0040 ) { return 2;}


 // contains restraint sensing below
 if (addr == 0x102e0040 ) { return 2;}
 if (addr == 0x102ec0cb ) { return 2;}
 if (addr == 0x102f0080 ) { return 2;}
 if (addr == 0x102f40cb ) { return 2;}
 if (addr == 0x10304058 ) { return 2;}
 if (addr == 0x10306099 ) { return 2;}
*/

 // contains restraint sensing below
/* 
 if (addr == 0x10308060 ) { return 2;}
 if (addr == 0x10312081 ) { return 2;}
 if (addr == 0x10320058 ) { return 2;}
 if (addr == 0x10330058 ) { return 2;} // could be restraint sensing
 if (addr == 0x10340080 ) { return 2;}
 if (addr == 0x1036a080 ) { return 2;}
 

 
 if (addr == 0x10376059 ) { return 2;}
 if (addr == 0x103880bc ) { return 2;}
 if (addr == 0x1038c040 ) { return 2;}
 if (addr == 0x10390040 ) { return 2;}
 if (addr == 0x103bc060 ) { return 2;}
 if (addr == 0x103d6060 ) { return 2;}
 if (addr == 0x103da060 ) { return 2;}
 if (addr == 0x10400058 ) { return 2;}
 if (addr == 0x1040a081 ) { return 2;}
 if (addr == 0x1040e0b9 ) { return 2;}
 if (addr == 0x104200cb ) { return 2;}
 if (addr == 0x10424040 ) { return 2;}
 if (addr == 0x1042c0cb ) { return 2;}
 */

 // should be clear below here (verified no DTCs on any module) 
/*
 if (addr == 0x10430097 ) { return 2;}
 if (addr == 0x104460cb ) { return 2;}
 if (addr == 0x10448060 ) { return 2;}
 if (addr == 0x1044a0cb ) { return 2;}
 if (addr == 0x1045c060 ) { return 2;}
 if (addr == 0x10466040 ) { return 2;}
 if (addr == 0x1046e040 ) { return 2;}
 if (addr == 0x1047a040 ) { return 2;}
 if (addr == 0x1058e0cb ) { return 2;}
 if (addr == 0x10600060 ) { return 2;}
 if (addr == 0x1060c040 ) { return 2;}
 if (addr == 0x106220cb ) { return 2;}
 if (addr == 0x10624099 ) { return 2;}
 if (addr == 0x1062c040 ) { return 2;}
 if (addr == 0x106340cb ) { return 2;}
 if (addr == 0x10644040 ) { return 2;}
 if (addr == 0x106b8040 ) { return 2;}
 if (addr == 0x106c2080 ) { return 2;}
 if (addr == 0x106d0080 ) { return 2;}
 if (addr == 0x106e0097 ) { return 2;}
 if (addr == 0x10704097 ) { return 2;}
 if (addr == 0x10722040 ) { return 2;}
*/







/*
 // end restraint sensing

 // Was able to clear all modules below here (not fresh restart)

 // start parking message 2 * not repeatable * 
 if (addr == 0x10724099 ) { return 2;}
 if (addr == 0x1072c0b9 ) { return 2;}
 if (addr == 0x1072e05b ) { return 2;} // Should be parking module message 2
 
 if (addr == 0x10730080 ) { return 2;}
 if (addr == 0x10734099 ) { return 2;}
 if (addr == 0x10754040 ) { return 2;}
 if (addr == 0x10758040 ) { return 2;}


 
 if (addr == 0x10760080 ) { return 2;}
 if (addr == 0x107640cb ) { return 2;}
 if (addr == 0x1077c040 ) { return 2;}
 if (addr == 0x10780040 ) { return 2;}
 if (addr == 0x107840cb ) { return 2;}
 if (addr == 0x107a0080 ) { return 2;}
 if (addr == 0x107b6080 ) { return 2;}

 // end parking message 2

 // Everything below here should be empty (not clean restart, but no modules have DTCs)


 if (addr == 0x107ba080 ) { return 2;}
 if (addr == 0x10806040 ) { return 2;}
 if (addr == 0x1080a040 ) { return 2;}
 if (addr == 0x10812060 ) { return 2;}
 if (addr == 0x10814099 ) { return 2;}
 if (addr == 0x1084a060 ) { return 2;}
 if (addr == 0x108680cb ) { return 2;}
 //start radio
 if (addr == 0x1086c0cb ) { return 2;}
 if (addr == 0x108700cb ) { return 2;} // this could be radio
 // end radio
*/


/*
 if (addr == 0x108e4097 ) { return 2;}


 if (addr == 0x108e8097 ) { return 2;}


 if (addr == 0x108f0097 ) { return 2;}
 if (addr == 0x108f2097 ) { return 2;}
 if (addr == 0x108f4097 ) { return 2;}

 if (addr == 0x10900097 ) { return 2;}
 if (addr == 0x10902097 ) { return 2;}
 if (addr == 0x10904097 ) { return 2;}
 if (addr == 0x10a240cb ) { return 2;}


// not fresh start, but no DTCs on any module

 if (addr == 0x10a2c0cb ) { return 2;}
 if (addr == 0x10aca040 ) { return 2;}
 if (addr == 0x10aca097 ) { return 2;}
 if (addr == 0x10ace060 ) { return 2;}
 if (addr == 0x10ad6080 ) { return 2;}
 if (addr == 0x10ae6080 ) { return 2;}


 if (addr == 0x10aec040 ) { return 2;}
 if (addr == 0x10b02097 ) { return 2;}
 if (addr == 0x10b02099 ) { return 2;}
 if (addr == 0x10b20097 ) { return 2;}

// end fresh start no dtcs on any modue 
*/










/*
 if (addr == 0x13ffe05b ) { return 2;}
 if (addr == 0x13ffe066 ) { return 2;}
 if (addr == 0x13ffe068 ) { return 2;}

 if (addr == 0x13ffe081 ) { return 2;}
 if (addr == 0x13ffe097 ) { return 2;}
 if (addr == 0x13ffe099 ) { return 2;}

 if (addr == 0x13ffe0af ) { return 2;}
 if (addr == 0x13ffe0b9 ) { return 2;}

 if (addr == 0x13ffe0bc ) { return 2;}
 if (addr == 0x13ffe0cb ) { return 2;}
 if (addr == 0x00000100 ) { return 2;}
 if (addr == 0x00000101 ) { return 2;}
 if (addr == 0x0000024c ) { return 2;}
 if (addr == 0x00000622 ) { return 2;}
 if (addr == 0x0000062d ) { return 2;}
 if (addr == 0x00000634 ) { return 2;}
 if (addr == 0x00000641 ) { return 2;}
 if (addr == 0x00000642 ) { return 2;}
 if (addr == 0x0000064c ) { return 2;}
 if (addr == 0x000007ec ) { return 2;}
 if (addr == 0x10254040 ) { return 2;}
 if (addr == 0x10260040 ) { return 2;}
 if (addr == 0x10288040 ) { return 2;}
 if (addr == 0x10336058 ) { return 2;}
 if (addr == 0x10380099 ) { return 2;}
 if (addr == 0x103a00bb ) { return 2;}
 if (addr == 0x103a40bb ) { return 2;}
 if (addr == 0x103a80bb ) { return 2;}
 if (addr == 0x1044e0bc ) { return 2;}
 if (addr == 0x106d4099 ) { return 2;}
 if (addr == 0x106f80bc ) { return 2;}
 if (addr == 0x1076c099 ) { return 2;}
 if (addr == 0x10774060 ) { return 2;}
 if (addr == 0x10a04060 ) { return 2;}
 if (addr == 0x10ac0097 ) { return 2;}
 if (addr == 0x10ac00bc ) { return 2;}
 if (addr == 0x10aca099 ) { return 2;}
 if (addr == 0x10aca0b9 ) { return 2;}
 if (addr == 0x10aca0bb ) { return 2;}
 if (addr == 0x10aca0bc ) { return 2;}
 if (addr == 0x10ae8060 ) { return 2;}
 if (addr == 0x10aec099 ) { return 2;}
 if (addr == 0x10aec0b9 ) { return 2;}
 if (addr == 0x10b020bc ) { return 2;}
 if (addr == 0x102d4040 ) { return 2;}
 if (addr == 0x103a6040 ) { return 2;}
 if (addr == 0x103d4040 ) { return 2;}
 if (addr == 0x103dc040 ) { return 2;}
 if (addr == 0x10428040 ) { return 2;}
 if (addr == 0x10444060 ) { return 2;}
 if (addr == 0x1046a040 ) { return 2;}
 if (addr == 0x10610040 ) { return 2;}
 if (addr == 0x10614040 ) { return 2;}
 if (addr == 0x10632040 ) { return 2;}
 if (addr == 0x10646040 ) { return 2;}
 if (addr == 0x1064a040 ) { return 2;}
 if (addr == 0x106f0040 ) { return 2;}
 if (addr == 0x106fc040 ) { return 2;}
 if (addr == 0x10728040 ) { return 2;}
 if (addr == 0x10768040 ) { return 2;}
 if (addr == 0x1076c040 ) { return 2;}
 if (addr == 0x10790040 ) { return 2;}
 if (addr == 0x10b0a060 ) { return 2;}


 if (addr == 0x00000244 ) { return 2;}
 if (addr == 0x00000631 ) { return 2;}
 if (addr == 0x00000644 ) { return 2;}
 if (addr == 0x10400040 ) { return 2;}
 if (addr == 0x10400060 ) { return 2;}
 if (addr == 0x10440099 ) { return 2;}
 if (addr == 0x10ac0099 ) { return 2;}
*/

// return 2;
//end block

 return -1;
 puts("addr: ");
 puth(addr);
 puts(" end\n");   



  }

  // Obj
  if (bus_num == 1) {
    return -1;
    // lka icon
    if (addr == 0x104c006c) {
      puts("\nSending LKA Icon: ");
      puth(to_fwd->RIR);
      puts("\n");
      //to_fwd->RDTR = 3;
      //to_fwd->RDLR = 0x00;
      //to_fwd->RDHR = 0;
      return 0;
    }
  }

  if (bus_num == 2) {
    return 0;
  }

  return -1;
}

void send_interceptor_status() {
    CAN_FIFOMailBox_TypeDef status;

    // Send HV wakeup on boot
    if (tick == 0) {
      status.RIR = (0x100 << 21) | 1;
      status.RDTR = 0;
      status.RDLR = 0;
      status.RDHR = 0;

      can_push(can_queues[2], &status);
      process_can(CAN_NUM_FROM_BUS_NUM(2));
    }

    status.RIR = (887 << 21) | 1;
    status.RDTR = 8;
    status.RDLR = 0x01020304;
    status.RDHR = 0x05060708;
    can_push(can_queues[1], &status);
    process_can(CAN_NUM_FROM_BUS_NUM(1));

    if ( can2_rx_cnt == 1) {
      CAN_FIFOMailBox_TypeDef stay_awake;
      stay_awake.RIR =  (0x620 << 21) | 1;
      stay_awake.RDTR = 0x8;
      stay_awake.RDLR = 0x00004000; 
      stay_awake.RDHR = 0x00000000;

      puts("generated stay awake: RIR:");
      puth(stay_awake.RIR);
      puts(" RDTR: ");
      puth(stay_awake.RDTR);
      puts(" RDLR: ");
      puth(stay_awake.RDLR);
      puts(" RDHR: ");
      puth(stay_awake.RDHR);
      puts("\n");
      can_push(can_queues[2], &stay_awake);
      process_can(CAN_NUM_FROM_BUS_NUM(2));
    }
}	

// CAN receive handlers
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
