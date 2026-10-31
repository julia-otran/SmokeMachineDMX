#include <Arduino.h>

#define RELAY_ACTUATOR_PIN 4
#define RELAY_DEBOUNCE_FRAMES 10
#define DMX_BAUD_RATE 250000
#define DMX_CHANNELS 512

uint16_t control_dmx_channel = 300;

void setup() {
  // See page 78
  // CLK_PER 3.3MHz
  int8_t sigrow_val = SIGROW_OSC20ERR5V;
  int32_t baud_reg_val = DMX_BAUD_RATE;

  baud_reg_val *= (1024 + sigrow_val);
  baud_reg_val /= 1024;
  
  USART0_BAUD = (int16_t) baud_reg_val;

  USART0_CTRLC = USART_SBMODE_2BIT_gc | USART_CHSIZE_8BIT_gc;
  USART0_CTRLB = USART_RXEN_bm;

  TCA0_SINGLE_INTCTRL = 0x1;

  // Page 203
  // ~66 uS
  TCA0_SINGLE_PER = (uint16_t)109U;
  TCA0_SINGLE_CTRLD = 0;
  TCA0_SINGLE_CTRLC = 0;
  TCA0_SINGLE_CTRLB = TCA_SINGLE_ALUPD_bm;

  PORTA_OUTCLR = 1 << RELAY_ACTUATOR_PIN;
  PORTA_DIRSET = 1 << RELAY_ACTUATOR_PIN;

  TCA0_SINGLE_CTRLA = 0;
  TCA0_SINGLE_CNT = (uint16_t)0;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL0_bm;
}

int16_t current_dmx_byte_num = 0;
uint8_t current_dmx_ignore_frame = 0;
uint8_t current_dmx_debounce_frame_count = 0;
uint8_t current_dmx_no_data_count = 0;
uint8_t relay_is_enabled = 0;
uint8_t relay_change_pending = 0;
uint8_t relay_should_be_enabled = 0;

ISR(TCA0_OVF_vect) {
  if (TCA0_SINGLE_INTFLAGS & 0x1) {
    TCA0_SINGLE_INTFLAGS = 0x1;
    USART0_STATUS = USART_WFB_bm;

    current_dmx_byte_num = 0;
    current_dmx_ignore_frame = 0;

    if (current_dmx_no_data_count < UINT8_MAX) {
      current_dmx_no_data_count += 1;
    }
  }
}

void loop() {
  uint8_t received_val = 0;
  uint8_t data_received = 0;

  if (USART0_STATUS & USART_RXCIF_bm) {
    TCA0_SINGLE_CTRLESET = TCA_SINGLE_CMD_UPDATE_gc;
    received_val = USART0_RXDATAL;
    data_received = 1;
    current_dmx_no_data_count = 0;
  }

  if (current_dmx_byte_num == 0 && received_val != 0) {
    current_dmx_ignore_frame = 1;
  }

  if (current_dmx_byte_num > DMX_CHANNELS) {
    current_dmx_ignore_frame = 1;
  }

  // This looses debounce when DMX CTRL Byte is not 0
  // Not that bad, because we don't know the size of the 
  // incoming DMX frame and don't know when it will end.
  // Such thing prevents the debouncing from getting stuck.
  if (
    data_received && 
    current_dmx_byte_num == 0 && 
    current_dmx_debounce_frame_count < UINT8_MAX
  ) {
    current_dmx_debounce_frame_count += 1;
  }

  if (data_received && current_dmx_byte_num == control_dmx_channel) {
    if (received_val >= 127) {
      relay_should_be_enabled = 1;
    } else {
      relay_should_be_enabled = 0;
    }
  }

  if (current_dmx_no_data_count >= UINT8_MAX) {
    relay_should_be_enabled = 0;
  }

  if (relay_is_enabled != relay_should_be_enabled) {
    if (current_dmx_debounce_frame_count < RELAY_DEBOUNCE_FRAMES) {
      current_dmx_debounce_frame_count = 0;
    }
    relay_is_enabled = relay_should_be_enabled;
    relay_change_pending = 1;
  }

  if (current_dmx_debounce_frame_count > RELAY_DEBOUNCE_FRAMES) {
    if (relay_change_pending) {
      relay_change_pending = 0;

      if (relay_is_enabled) {
        PORTA_OUTSET = 1 << RELAY_ACTUATOR_PIN;
      } else {
        PORTA_OUTCLR = 1 << RELAY_ACTUATOR_PIN;
      }
    }
  }

  if (data_received && !current_dmx_ignore_frame) {
    current_dmx_byte_num += 1;
  }
}
