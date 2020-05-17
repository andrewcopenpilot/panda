#include "config.h"
#include "obj/gitversion.h"

// ********************* includes *********************

#include "libc.h"
//#include "safety.h"
#include "provision.h"

#include "drivers/drivers.h"

#include "drivers/llgpio.h"
#include "gpio.h"

#include "drivers/uart.h"
#include "drivers/adc.h"
#include "drivers/usb.h"
#include "drivers/can.h"
#include "drivers/timer.h"


// ********************* serial debugging *********************

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv)) {
    putc(ring, rcv);
  }
}

// ***************************** USB port *****************************

int is_enumerated = 0;

void usb_cb_enumeration_complete() {
  puts("USB enumeration complete\n");
  is_enumerated = 1;
}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, int hardwired) {
  int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xd0: fetch serial number
    case 0xd0:
        // addresses are OTP
        if (setup->b.wValue.w == 1) {
          memcpy(resp, (void *)0x1fff79c0, 0x10);
          resp_len = 0x10;
        } else {
          get_provision_chunk(resp);
          resp_len = PROVISION_CHUNK_LEN;
        }
      break;
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      // so it's blocked over wifi
      switch (setup->b.wValue.w) {
        case 0:
          if (hardwired) {
            puts("-> entering bootloader\n");
            enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
            NVIC_SystemReset();
          }
          break;
        case 1:
          puts("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
      }
      break;
    // **** 0xd6: get version
    case 0xd6:
      COMPILE_TIME_ASSERT(sizeof(gitversion) <= MAX_RESP_LEN)
      memcpy(resp, gitversion, sizeof(gitversion));
      resp_len = sizeof(gitversion)-1;
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) break;
      if (ur == &esp_ring) uart_dma_drain();
      // read
      while ((resp_len < min(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xe1: uart set baud rate
    case 0xe1:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) break;
      uart_set_baud(ur->uart, setup->b.wIndex.w);
      break;
    // **** 0xe2: uart set parity
    case 0xe2:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) break;
      switch (setup->b.wIndex.w) {
        case 0:
          // disable parity, 8-bit
          ur->uart->CR1 &= ~(USART_CR1_PCE | USART_CR1_M);
          break;
        case 1:
          // even parity, 9-bit
          ur->uart->CR1 &= ~USART_CR1_PS;
          ur->uart->CR1 |= USART_CR1_PCE | USART_CR1_M;
          break;
        case 2:
          // odd parity, 9-bit
          ur->uart->CR1 |= USART_CR1_PS;
          ur->uart->CR1 |= USART_CR1_PCE | USART_CR1_M;
          break;
        default:
          break;
      }
      break;
    // **** 0xe4: uart set baud rate extended
    case 0xe4:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) break;
      uart_set_baud(ur->uart, (int)setup->b.wIndex.w*300);
      break;
    // **** 0xf2: Clear UART ring buffer.
    case 0xf2:
      {
        uart_ring * rb = get_ring_by_number(setup->b.wValue.w);
        if (rb) {
          puts("Clearing UART queue.\n");
          clear_uart_buff(rb);
        }
        break;
      }
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

// ***************************** main code *****************************

void __initialize_hardware_early() {
  early();
}

void __attribute__ ((noinline)) enable_fpu() {
  // enable the FPU
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
}

int main() {
  // shouldn't have interrupts here, but just in case
  __disable_irq();

  // init early devices
  clock_init();
  periph_init();
  detect();

  // print hello
  puts("\n\n\n************************ MAIN START ************************\n");

  // detect the revision and init the GPIOs
  puts("config:\n");
  puts(revision == PANDA_REV_C ? "  panda rev c\n" : "  panda rev a or b\n");
  puts(has_external_debug_serial ? "  real serial\n" : "  USB serial\n");
  puts(is_giant_panda ? "  GIANTpanda detected\n" : "  not GIANTpanda\n");
  puts(is_grey_panda ? "  gray panda detected!\n" : "  white panda\n");
  puts(is_entering_bootmode ? "  ESP wants bootmode\n" : "  no bootmode\n");
  gpio_init();

  // panda has an FPU, let's use it!
  enable_fpu();

  // enable main uart if it's connected
  if (has_external_debug_serial) {
    // WEIRDNESS: without this gate around the UART, it would "crash", but only if the ESP is enabled
    // assuming it's because the lines were left floating and spurious noise was on them
    uart_init(USART2, 115200);
  }

  // init microsecond system timer
  // increments 1000000 times per second
  // generate an update to set the prescaler
  TIM2->PSC = 48-1;
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->EGR = TIM_EGR_UG;
  // use TIM2->CNT to read

  
  // enable USB
  usb_init();

  set_esp_mode(ESP_DISABLED);

  can_init_all();
  delay(10000000);
  can_init_hw();

  adc_init();

  puts("**** INTERRUPTS ON ****\n");

  __enable_irq();

  ttl_timer_init();

  // if the error interrupt is enabled to quickly when the CAN bus is active
  // something bad happens and you can't connect to the device over USB
  delay(10000000);

  CAN1->IER |= CAN_IER_ERRIE | CAN_IER_LECIE;

  // LED should keep on blinking all the time
  uint64_t cnt = 0;


  set_led(LED_BLUE, 1);
  set_led(LED_RED, 0);
  set_led(LED_GREEN, 0);

  for (cnt=0;;cnt++) {
    //  if (cnt % 16384 == 0) {
    if (cnt % 33554432 == 0) {
      //puts("cnt: "); puth(cnt);
      puts(" rx0: "); puth(can0_rx_cnt);
      puts(" tx0: "); puth(can0_tx_cnt);
      puts(" rx2: "); puth(can2_rx_cnt);
      puts(" tx2: "); puth(can2_tx_cnt);

      puts(" err: "); puth(can_err_cnt);
      puts(" mboxfull0: "); puth(can0_mailbox_full_cnt);
      puts(" mboxfull2: "); puth(can1_mailbox_full_cnt);
      puts("\n");

      CAN_TypeDef *CANZERO = CANIF_FROM_CAN_NUM(0);
      puts("CAN1:  ");
      puts("MSR:");
      puth(CANZERO->MSR);
      puts(" TSR:");
      puth(CANZERO->TSR);
      puts(" RF0R:");
      puth(CANZERO->RF0R);
      puts(" RF1R:");
      puth(CANZERO->RF1R);
      puts(" ESR:");
      puth(CANZERO->ESR);
      puts("\n");

      CAN_TypeDef *CANTWO = CANIF_FROM_CAN_NUM(2);
      puts("CAN3:  ");
      puts("MSR:");
      puth(CANTWO->MSR);
      puts(" TSR:");
      puth(CANTWO->TSR);
      puts(" RF0R:");
      puth(CANTWO->RF0R);
      puts(" RF1R:");
      puth(CANTWO->RF1R);
      puts(" ESR:");
      puth(CANTWO->ESR);
      puts("\n");
    }
  }

  return 0;
}
