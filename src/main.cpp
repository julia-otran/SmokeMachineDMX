#include <Arduino.h>
#include <EEPROM.h>

#define RELAY_ACTUATOR_PIN 5
#define RELAY_DEBOUNCE_FRAMES 10
#define DMX_BAUD_RATE 250000L
#define F_BAUD_DIV ((64L * 20000000L) / 16L)
#define DMX_CHANNELS 512
#define EEPROM_DEFAULT_CONTROL_DMX_CHANNEL 192
#define DMX_CHANNEL_PROGRAM_PORTA_PIN 6

struct EEPROMData {
  uint16_t control_dmx_channel;
};

uint16_t control_dmx_channel = EEPROM_DEFAULT_CONTROL_DMX_CHANNEL;
EEPROMData eeprom_data;

void setup() {
  EEPROM.get(0, eeprom_data);

  if (eeprom_data.control_dmx_channel <= 0 || eeprom_data.control_dmx_channel > DMX_CHANNELS) {   
    eeprom_data.control_dmx_channel = EEPROM_DEFAULT_CONTROL_DMX_CHANNEL;
    EEPROM.put(0, eeprom_data);
  }

  control_dmx_channel = eeprom_data.control_dmx_channel;

  PORTB_DIRSET = 1 << 2;
  PORTB_DIRCLR = 1 << 1;

  PORTA_OUTCLR = 1 << RELAY_ACTUATOR_PIN;
  PORTA_DIRSET = 1 << RELAY_ACTUATOR_PIN;

  // See page 78
  int16_t sigrow_val = SIGROW_OSC20ERR5V;
  int32_t baud_reg_val = DMX_BAUD_RATE;

  baud_reg_val *= (1024 + sigrow_val);
  baud_reg_val /= 1024;

  USART0_BAUD = (uint16_t) (F_BAUD_DIV / baud_reg_val);

  USART0_CTRLA = 0;
  USART0_CTRLC = USART_SBMODE_2BIT_gc | USART_CHSIZE_8BIT_gc;
  USART0_CTRLB = USART_RXEN_bm | USART_TXEN_bm;


  // TCA0 used for DMX no-signal timeout
  // ~2s
  TCA0_SINGLE_PER = 40000;
  TCA0_SINGLE_CTRLD = 0;
  TCA0_SINGLE_CTRLC = 0;
  TCA0_SINGLE_CTRLB = TCA_SINGLE_ALUPD_bm;
  TCA0_SINGLE_CTRLA = 0;
  TCA0_SINGLE_CNT = (uint16_t)0;
  TCA0_SINGLE_INTCTRL = 0x1;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV1024_gc;

  // TCB0 used for DMX break detection, ~66 uS
  // 66 uS
  TCB0_CCMP = (uint16_t)660U;
  TCB0_CNT = 0;
  TCB0_INTCTRL = TCB_CAPT_bm;
  TCB0_CTRLA = TCB_ENABLE_bm | TCB_CLKSEL_CLKDIV2_gc;
  TCB0_CTRLB = 0;
}

int16_t current_dmx_byte_num = 0;
uint8_t current_dmx_ignore_frame = 0;
uint8_t current_dmx_debounce_frame_count = 0;
uint8_t current_dmx_no_signal = 0;
uint8_t current_dmx_count_channels_with_value = 0;
uint8_t current_dmx_eligible_channel = 0;
uint8_t relay_is_enabled = 0;
uint8_t relay_change_pending = 0;
uint8_t relay_should_be_enabled = 0;

ISR(TCA0_OVF_vect) {
  if (TCA0_SINGLE_INTFLAGS & 0x1) {
    TCA0_SINGLE_INTFLAGS = 0x1;
    current_dmx_no_signal = 1;
  }
}

ISR(TCB0_INT_vect) {
  if (TCB0_INTFLAGS & TCB_CAPT_bm) {
    TCB0_INTFLAGS = TCB_CAPT_bm;
    current_dmx_byte_num = 0;
    current_dmx_ignore_frame = 0;
  }
}

void loop() {
  uint8_t received_val = 0;
  uint8_t received_flag = 0;
  uint8_t data_received = 0;
  uint8_t reprogram_flag = PORTA_IN & (1 << DMX_CHANNEL_PROGRAM_PORTA_PIN);

  if (USART0_STATUS & USART_RXCIF_bm) {
    TCB0_CNT = (uint16_t)0;
    
    received_flag = USART0_RXDATAH;
    received_val = USART0_RXDATAL;
    data_received = 1;
  }

  if (data_received && current_dmx_byte_num == 0 && received_flag & USART_FERR_bm) {
    return;
  }

  if (data_received && current_dmx_byte_num == 0 && received_val != 0) {
    current_dmx_ignore_frame = 1;
  }

  if (data_received && current_dmx_byte_num > DMX_CHANNELS) {
    current_dmx_ignore_frame = 1;
  }

  if (received_flag & USART_FERR_bm) {
    current_dmx_ignore_frame = 1;
  }

  if (received_flag & USART_BUFOVF_bm) {
    current_dmx_ignore_frame = 1;
  }

  if (current_dmx_ignore_frame) {
    current_dmx_eligible_channel = 0;
    current_dmx_count_channels_with_value = 0;
  }

  if (data_received && current_dmx_byte_num == 0 && !current_dmx_ignore_frame) {
    current_dmx_no_signal = 0;
    TCA0_SINGLE_CNT = (uint16_t)0;
  }

  if (
    !current_dmx_ignore_frame &&
    !current_dmx_no_signal &&
    current_dmx_byte_num > 0 &&
    received_val != 0 &&
    data_received &&
    current_dmx_eligible_channel != current_dmx_byte_num
  ) {
    current_dmx_count_channels_with_value += 1;
    current_dmx_eligible_channel = current_dmx_byte_num;
  }

  if (data_received && current_dmx_byte_num == 0 && current_dmx_count_channels_with_value != 1) {
    current_dmx_eligible_channel = 0;
    current_dmx_count_channels_with_value = 0;
  }

  if (
    !current_dmx_ignore_frame &&
    !current_dmx_no_signal &&
    current_dmx_byte_num == 0 &&
    current_dmx_count_channels_with_value == 1 &&
    current_dmx_eligible_channel &&
    reprogram_flag &&
    control_dmx_channel != current_dmx_eligible_channel
  ) {
    control_dmx_channel = current_dmx_eligible_channel;
    eeprom_data.control_dmx_channel = control_dmx_channel;
    EEPROM.put(0, eeprom_data);
  }

  if (
    current_dmx_eligible_channel &&
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

  if (relay_is_enabled != relay_should_be_enabled) {
    if (current_dmx_debounce_frame_count < RELAY_DEBOUNCE_FRAMES) {
      current_dmx_debounce_frame_count = 0;
    }
    relay_is_enabled = relay_should_be_enabled;
    relay_change_pending = 1;
  }

  if (current_dmx_no_signal) {
    relay_should_be_enabled = 0;
    relay_is_enabled = 0;
    relay_change_pending = 1;
    current_dmx_debounce_frame_count = UINT8_MAX;
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
