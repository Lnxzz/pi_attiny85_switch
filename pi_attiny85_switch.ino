/**
 * ATTiny85 / Digispark - Raspberry Pi Power Control
 *
 * Version: 1.0
 * Date:    2024/03/17
 * Author:  E. Heerschop
 *
 *
 * ATTiny85 pin assignment:
 *
 * 1 ADC0 POWER BUTTON            (analog input)
 * 3 PB4  SERIAL TX               (digital output)
 * 2 PB3  SERIAL RX               (digital input)
 * 7 PB2  DISPLAY CLOCK           (digital output)
 * 6 PB1  DISPLAY_DIO / POWER LED (digital output)
 * 5 PB0  RELAY SWITCH            (digital output)
 *
 *                       ATTiny85
 *    Reset, ADC0, PB5  1   o    8  VCC 5 Volts
 *           ADC3, PB3  2        7  PB2, ADC1, I2C SCL, SPI SCK
 *           ADC2, PB4  3        6  PB1, PWM, SPI MISO
 *              Ground  4        5  PB0, PWM, I2C SDA, SPI MOSI
 *
 * 
 * Raspberry Pi pin assignment:
 * UART TX         - RPi GPIO 14 / TXD / pin  8  - serial send
 * UART RX         - RPi GPIO 15 / RXD / pin 10  - serial receive
 *
 * The button pin is connected with ADC0, which is also the reset pin, to prevent the ATTiny85 to reset, 
 * a voltage divider is used with a 4.7K resister connected with ground and a 1K resistor connected with VCC.
 * The switch point is around 930 on the scale between 0 and 1023.
 *
 * The UART TX pin of the raspberry pi is directly connected with the RX pin on the ATTiny85.
 * The TX pin on the ATTiny85 is connected to the UART RX pin of the raspberry pi using a voltage divider.
 *
 * PB1 and PB2 can be used to drive a TM1637 based display
 *
 * PB0 is used to trigger a 5V relay switch module
 *
 *
 * This is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <TM1637Display.h>
#include <SoftSerial.h>
#include <TinyPinChange.h>


// timings
const unsigned long BLINK_DELAY_MS = 250;      // delay time in ms for led blinking
const unsigned long BUTTON_DELAY_MS = 5;       // delay time in ms for sampling the power button
const unsigned long SERIAL_DELTA_MS = 1000;    // min. time in ms before treating serial data as a new command
const unsigned long SHORT_PRESS_MS = 2000;     // max. delay time in ms for a short press on the power button
const unsigned long LONG_PRESS_MS = 3000;      // min. delay time in ms for a long press on the power button
const unsigned long WAIT_TIMEOUT_MS = 30000;   // timeout in ms to wait for shutdown signal
const unsigned long POWEROFF_DELAY_MS = 2000;  // delay time in ms to switch the relay off after power off detection or wait timeout

// pin assignment
const uint8_t RELAY_PIN = PB0;      // PB0 pin connected with the relay switching the raspberry pi on and off
const uint8_t LED_PIN = PB1;        // PB1 pin connected with the power led
const uint8_t DISPLAY_DIO = PB1;    // use PB1 pin as DIO for a TM1637 display (note: shared with the led pin)
const uint8_t DISPLAY_CLK = PB2;    // use PB2 pin as CLK for a TM1637 display
const uint8_t SERIAL_RX_PIN = PB3;  // PB3 pin as software serial receive
const uint8_t SERIAL_TX_PIN = PB4;  // PB4 pin as software serial sent
const uint8_t BUTTON_PIN = 0;       // ADC0 pin connected with the power button

// relay states
const uint8_t RELAY_ON = 0;        // set to 0 when relay is normally open; set to 1 when normally closed
const uint8_t RELAY_OFF = 1;       // set to 1 when relay is normally open; set to 0 when normally closed
uint8_t relay_status = RELAY_OFF;  // relay status: set to RELAY_ON or RELAY_OFF

// possible display/led modes
const uint8_t LED_MODE = 0;          // show status using a led
const uint8_t TM1637_MODE = 1;       // show status on TM1637 display
uint8_t display_mode = LED_MODE;     // use TM1637_MODE (debug) or LED_MODE (normal)

// possible led states
const uint8_t LED_OFF = 0;             // led continuous off
const uint8_t LED_ON = 1;              // led continuous on
const uint8_t LED_BLINK_ONCE_OFF = 2;  // blink led once off state
const uint8_t LED_BLINK_ONCE_ON = 3;   // blink led once on state
const uint8_t LED_BLINK_CONT_OFF = 4;  // blink led continuous off state
const uint8_t LED_BLINK_CONT_ON = 5;   // blink led continuous on state

uint8_t led_status = LED_OFF;      // led status: one of possible led states
uint8_t old_led_status = LED_OFF;  // previous led state
unsigned long led_period_ms;       // led blink period in ms
unsigned long led_timestamp;       // timestamp for led blinking

// possible power states
const uint8_t POWERED_OFF = 0;
const uint8_t POWERING_ON = 1;
const uint8_t POWERED_ON = 2;
const uint8_t POWERING_OFF = 3;
const uint8_t WAIT_FOR_SHUTDOWN = 4;
const uint8_t HARD_SHUTDOWN = 5;

uint8_t power_status = POWERED_OFF;  // power status: 0=power off; 1=powering on; 2=powered on; 3=powering off; 4=waiting for shutdown signal; 5=hard shutdown

int poweroff_aval = 0;  // analog power value read from ADC3 (RX) pin

// button states
const uint8_t BUTTON_RELEASED = 0;
const uint8_t BUTTON_PRESSED = 1;
const uint8_t BUTTON_IDLE = 2;
const uint8_t BUTTON_WAIT_RELEASE = 3;
const uint8_t BUTTON_WAIT_RELEASED = 4;

// button input handling
const int BUTTON_REFERENCE_VALUE = 930;    // analog reference value
uint8_t button_status = BUTTON_IDLE;       // current button state
int button_states;                         // array of last button states
int button_aval = 0;                       // analog button value read
int button_dval = BUTTON_RELEASED;         // digital button value (BUTTON_RELEASED or BUTTON_PRESSED)
unsigned long last_button_millis = 0;      // timestamp of previous read value
unsigned long button_pressed_millis = 0;   // timestamp when a press on the button has been confirmed
unsigned long button_released_millis = 0;  // timestamp when the release of the button has been confirmed

// timestamps
unsigned long shutdown_timestamp = 0;      // timestamp to power off
unsigned long timestamp = 0;               // reference timestamp updated every loop iteration

// TM1637 display settings
const uint8_t DISPLAY_BRIGHTNESS = 0x07;   // set display brightness between 0..7

// display segments:
//
//           +-a-+
//           f   b
//           +-g-+
//           e   c
//           +-d-+
//
const uint8_t DISPLAY_MSG_OFF[] = {
  SEG_C | SEG_D | SEG_E | SEG_G,          // o
  SEG_A | SEG_E | SEG_F | SEG_G,          // f
  SEG_A | SEG_E | SEG_F | SEG_G,          // f
  0x00                                    //
};                                        //
const uint8_t DISPLAY_MSG_ON[] = {
  SEG_C | SEG_D | SEG_E | SEG_G,          // o
  SEG_C | SEG_E | SEG_G,                  // n
  0x00,                                   //
  0x00                                    //
};                                        //
const uint8_t DISPLAY_MSG_BOOT[] = {
  SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,  // b
  SEG_C | SEG_D | SEG_E | SEG_G,          // o
  SEG_C | SEG_D | SEG_E | SEG_G,          // o
  SEG_D | SEG_E | SEG_F | SEG_G           // t
};                                        //
const uint8_t DISPLAY_MSG_SHUTDOWN[] = {
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,  // s
  SEG_C | SEG_E | SEG_F | SEG_G,          // h
  SEG_C | SEG_D | SEG_E,                  // u
  SEG_D | SEG_E | SEG_F | SEG_G           // t
};                                        //

TM1637Display display(DISPLAY_CLK, DISPLAY_DIO);

// software serial port settings and commands
const long BAUD_RATE = 9600;           // serial baud rate
const int CMD_NULL = 0x00;             // command not available
const int CMD_BOOTOK = 0x42;           // 'BOOTOK'   - OS boot done
const int CMD_SHUTDOWN = 0x53;         // 'SHUTDOWN' - shutdown initiated by software
const int CMD_LED = 0x4C;              // 'LED'      - use led for notification
const int CMD_DISPLAY = 0x44;          // 'DISPLAY'  - use display for notification
const int CMD_STATUS = 0x47;           // 'STATUS'   - get current power status
const int CMD_MILLIS = 0x4D;           // 'MILLIS'   - get elapsed time in milliseconds

#define BUF_SIZE 16
char serial_buffer[BUF_SIZE];          // serial input buffer
int serial_idx = 0;                    // index within the serial input buffer
int serial_data = CMD_NULL;            // last data received on serial port
int command = CMD_NULL;                // current command to execute
unsigned long serial_last_millis = 0;  // timestamp when data was last received on the serial port

SoftSerial swserial(SERIAL_RX_PIN, SERIAL_TX_PIN);


// setup pins, initial state and ensure we are powered off
void setup() {
  analogReference(DEFAULT);            // set analog reference
  pinMode(PB0, OUTPUT);                // relay
  digitalWrite(PB0, RELAY_OFF);        // ensure relay is of asap
  pinMode(PB1, OUTPUT);                // LED / display DIO
  pinMode(PB2, OUTPUT);                // display CLK
  pinMode(PB3, INPUT);                 // RX from PI
  pinMode(PB4, OUTPUT);                // TX to PI
  pinMode(PB5, INPUT);                 // power button
  power_off(false);                    // set to initial state: POWERED_OFF
}

// power pi on
void power_on() {
  if (relay_status == RELAY_OFF) {
    if (display_mode == LED_MODE) {
      set_led(LED_BLINK_CONT_OFF, BLINK_DELAY_MS);
    } else {
      display.setBrightness(DISPLAY_BRIGHTNESS, true);
      display.clear();
      display.setSegments(DISPLAY_MSG_BOOT);
    }
    digitalWrite(RELAY_PIN, RELAY_ON);
    relay_status = RELAY_ON;
    swserial.begin(BAUD_RATE);
  }
}

// power pi off
void power_off(bool verbose) {
  if (verbose && display_mode == TM1637_MODE) {
    display.showNumberDec(poweroff_aval, false);
    delay(POWEROFF_DELAY_MS);
  }
  if (display_mode == LED_MODE) {
    if (verbose) {
      delay(POWEROFF_DELAY_MS);
    }
    set_led(LED_OFF, 0);
  } else {
    if (verbose) {
      display.setSegments(DISPLAY_MSG_OFF);
      delay(POWEROFF_DELAY_MS);
    }
    display.setBrightness(DISPLAY_BRIGHTNESS, false);
    display.clear();
  }
  digitalWrite(RELAY_PIN, RELAY_OFF);
  relay_status = RELAY_OFF;
  power_status = POWERED_OFF;
  serial_data = CMD_NULL;
  command = CMD_NULL;
  swserial.end();
}

// update led
// pre: display_mode = LED_MODE
void update_led() {
  if (led_status != old_led_status) {
    if (led_status == LED_OFF) {
      digitalWrite(LED_PIN, LOW);
      old_led_status = led_status;
    } else if (led_status == LED_ON) {
      digitalWrite(LED_PIN, HIGH);
      old_led_status = led_status;
    } else {
      if (timestamp >= led_timestamp + led_period_ms) {
        if (led_status == LED_BLINK_ONCE_OFF) {
          digitalWrite(LED_PIN, LOW);
          led_status = LED_BLINK_ONCE_ON;
        } else if (led_status == LED_BLINK_ONCE_ON) {
          digitalWrite(LED_PIN, HIGH);
          led_status = old_led_status;
          old_led_status = LED_BLINK_ONCE_ON;
        } else if (led_status == LED_BLINK_CONT_OFF) {
          digitalWrite(LED_PIN, LOW);
          led_status = LED_BLINK_CONT_ON;
        } else if (led_status == LED_BLINK_CONT_ON) {
          digitalWrite(LED_PIN, HIGH);
          led_status = LED_BLINK_CONT_OFF;
        }
        led_timestamp = timestamp;
      }
    }
  }
}

// set new led status and blink period
// pre: display_mode = LED_MODE
void set_led(uint8_t status, unsigned long period_ms) {
  led_status = status;
  led_period_ms = period_ms;
  led_timestamp = timestamp;
  pinMode(PB1, OUTPUT);
  update_led();
}

// read and update the state of the power button and the power status
void check_power_button() {
  if (timestamp - last_button_millis >= BUTTON_DELAY_MS) {
    // take a sample
    button_aval = analogRead(BUTTON_PIN);
    if (button_aval < BUTTON_REFERENCE_VALUE) {
      button_dval = BUTTON_PRESSED;
    } else {
      button_dval = BUTTON_RELEASED;
    }

    // process last set of samples
    int sum_of_states = 0;
    button_states <<= 1;
    button_states |= button_dval;
    sum_of_states = __builtin_popcount(button_states);

    // update button status
    if (button_status == BUTTON_IDLE && sum_of_states >= 10 && (timestamp - button_released_millis >= SHORT_PRESS_MS || button_released_millis == 0)) {
      // button press confirmed
      button_pressed_millis = timestamp;
      button_status = BUTTON_PRESSED;
      button_states = 0;
    } else if (button_status == BUTTON_PRESSED && sum_of_states == 0) {
      // button release confirmed
      button_released_millis = timestamp;
      button_status = BUTTON_RELEASED;
      button_states = 0;
    } else if (button_status == BUTTON_RELEASED && timestamp - button_released_millis >= SHORT_PRESS_MS) {
      // button released a short press ago: wait for next press of the button
      button_status = BUTTON_IDLE;
      button_states = 0;
    } else if (button_status == BUTTON_WAIT_RELEASE && sum_of_states == 0) {
      // button release confirmed while waiting to release
      button_released_millis = timestamp;
      button_status = BUTTON_WAIT_RELEASED;
      button_states = 0;
    } else if (button_status == BUTTON_WAIT_RELEASED && timestamp - button_released_millis >= SHORT_PRESS_MS) {
      // button long pressed released: wait for next press of the button
      button_status = BUTTON_IDLE;
      button_states = 0;
    }

    // keep timestamp for next iteration
    last_button_millis = timestamp;
  }
}

// check if the pi has been shutdown completely
// pre: stop software serial
bool check_shutdown() {
  // use P03/ADC03 to check voltage on pin GPIO14 / TX on the PI
  poweroff_aval = analogRead(SERIAL_RX_PIN);
  return ((poweroff_aval / 1024.0) < 0.1);
}

// reset the serial input buffer
void reset_buffer() {
  for (int i = 0; i < BUF_SIZE; i++) {
    serial_buffer[i] = 0;
  }
  serial_idx = 0;
}

// check if the given data is at the start of the serial input buffer
bool starts_in_buffer(const char* data, int length) {
  int idx = 0;
  bool found = false;
  while (serial_buffer[idx] == data[idx] && idx < serial_idx && !found) {
    idx++;
    if (idx == length) {
      found = true;
    }
  }
  return found;
}

// check if the given data is part of the serial input buffer
// using the Knuth-Morris-Pratt algorithm 
// (see: https://en.wikipedia.org/wiki/Knuth%E2%80%93Morris%E2%80%93Pratt_algorithm)
// note: removed to reduce memory usage
/*
int has_in_buffer(const char* data, int data_len, int start=0) {
  int i=0;
  int j=-1;
  int lps[data_len];
  lps[0] = -1;

  // prepare longest prefix array
  while (i < data_len) {
    while (j > -1 && data[i] != data[j]) {
      j = lps[j];
    }
    i++;
    j++;
    if (data[i] == data[j]) {
      lps[i] = lps[j];
    } else {
      lps[i] = j;
    }
  }

  // search data in serial buffer
  i=0;
  j=start;
  while (j < serial_idx) {
    while (i > -1 && data[i] != serial_buffer[j]) {
      i = lps[i];
    }
    i++;
    j++;
    if (i >= data_len) {
      return (j - i);
    }
  }
  return -1;
}
*/

// check and read commands from the software serial port
void check_serial_port() {
  if (swserial.available() > 0 && command == CMD_NULL) {
    serial_data = swserial.read();
    // reset buffer if SERIAL_DELTA_MS passed
    if (timestamp - serial_last_millis >= SERIAL_DELTA_MS) {
      reset_buffer();
    }
    // reset buffer index to prevent overflow
    if (serial_idx + 1 >= BUF_SIZE) {
      serial_idx = 0;
    }
    // check if we have a valid character
    if ((serial_data > 0x1F && serial_data < 0x7F) || serial_data == 0x0A || serial_data == 0x0D) {
      // add character to buffer
      serial_buffer[serial_idx] = (char)(serial_data & 0xFF);
      serial_idx++;
      // check if we have received a command
      if (starts_in_buffer("BOOTOK", 6)) {
        reset_buffer();
        command = CMD_BOOTOK;
      } else if (starts_in_buffer("SHUTDOWN", 8)) {
        reset_buffer();
        command = CMD_SHUTDOWN;
      } else if (starts_in_buffer("LED", 3)) {
        reset_buffer();
        command = CMD_LED;
      } else if (starts_in_buffer("DISPLAY", 7)) {
        reset_buffer();
        command = CMD_DISPLAY;
      } else if (starts_in_buffer("STATUS", 6)) {
        reset_buffer();
        command = CMD_STATUS;
      } else if (starts_in_buffer("MILLIS", 6)) {
        reset_buffer();
        command = CMD_MILLIS;        
      } else if (starts_in_buffer(" ", 1)) {
        reset_buffer();
      } else if (starts_in_buffer("\n", 1)) {
        reset_buffer();
      } else if (starts_in_buffer("\r", 1)) {
        reset_buffer();
      } else if (starts_in_buffer("\t", 1)) {
        reset_buffer();
      }
    }
    serial_last_millis = timestamp;
  }
}

// flush serial input buffer
void serial_flush_input() {
  while (swserial.available() > 0) {
    serial_data = swserial.read();
  }
}

// send power status to the software serial port
void send_power_status() {
  if (power_status == POWERED_OFF) {
    swserial.println("powered off");
  } else if (power_status == POWERING_ON) {
    swserial.println("powering on");
  } else if (power_status == POWERED_ON) {
    swserial.println("powered on");
  } else if (power_status == POWERING_OFF) {
    swserial.println("powering off");
  } else if (power_status == WAIT_FOR_SHUTDOWN) {
    swserial.println("wait for shutdown");
  } else if (power_status == HARD_SHUTDOWN) {
    swserial.println("hard shutdown");
  } else {
    swserial.println("unknown");
  }
}

// main loop
void loop() {

  // update reference timestamp
  timestamp = millis();

  // update the power led
  if (display_mode == LED_MODE) {
    update_led();
  }

  // check and read commands from the software serial port
  check_serial_port();

  // handle serial command when available
  if (command != CMD_NULL) {
    // flush serial input buffer
    serial_flush_input();
    if (command == CMD_BOOTOK) {
      // boot sequence completed
      // confirm with bootok message
      swserial.println("bootok");
      // confirm with led continous on or "on" message on the display
      if (display_mode == LED_MODE) {
        set_led(LED_ON, 0);
      } else {
        display.setSegments(DISPLAY_MSG_ON);
      }
      // update power status to powered on
      power_status = POWERED_ON;
    } else if (command == CMD_SHUTDOWN) {
      // received software shutdown command: update power status
      swserial.println("led");
      power_status = POWERING_OFF;
    } else if (command == CMD_LED) {
      // turn on led, turn off display
      swserial.println("led");
      display_mode = LED_MODE;
      display.setBrightness(DISPLAY_BRIGHTNESS, false);
      display.clear();
      if (power_status == POWERING_ON) {
        set_led(LED_BLINK_CONT_OFF, BLINK_DELAY_MS);
      } else if (power_status == POWERED_ON) {
        set_led(LED_ON, 0);
      } else {
        set_led(LED_OFF, 0);
      }
    } else if (command == CMD_DISPLAY) {
      // turn on display, turn off led
      swserial.println("display");
      display_mode = TM1637_MODE;
      set_led(LED_OFF, 0);
      display.setBrightness(DISPLAY_BRIGHTNESS, true);
      display.clear();
      if (power_status == POWERING_ON) {
        display.setSegments(DISPLAY_MSG_BOOT);
      } else if (power_status == POWERED_ON) {
        display.setSegments(DISPLAY_MSG_ON);
      }
    } else if (command == CMD_STATUS) {
      // return power status
      send_power_status();
    } else if (command == CMD_MILLIS) {
      // send elapsed time in milliseconds
      swserial.println(timestamp, DEC);
    }
    // flush serial output
    swserial.flush();
    command = CMD_NULL;
  }

  // check the status of the power button
  check_power_button();

  // update state based on the button status and the time the button is pressed
  if (button_status == BUTTON_PRESSED && timestamp - button_pressed_millis >= LONG_PRESS_MS) {
    // long button press detected
    power_status = HARD_SHUTDOWN;
    button_status = BUTTON_WAIT_RELEASE;
  } else if (button_status == BUTTON_RELEASED && button_released_millis - button_pressed_millis <= SHORT_PRESS_MS) {
    // short button press detected
    if (power_status == POWERED_OFF) {
      power_status = POWERING_ON;
    } else if (power_status == POWERING_ON || power_status == POWERED_ON) {
      power_status = POWERING_OFF;
    }
    button_status = BUTTON_IDLE;
  }

  // power state handling
  if (power_status == POWERING_ON) {

    // switch relay to power on
    power_on();

  } else if (power_status == POWERED_ON) {

    // powered on: running normally

  } else if (power_status == POWERING_OFF) {

    // request pi to shutdown
    swserial.println("shutdown");
    swserial.flush();

    // wait for final shutdown (or timeout)
    if (display_mode == LED_MODE) {
      set_led(LED_BLINK_CONT_OFF, BLINK_DELAY_MS);
    } else {
      display.setSegments(DISPLAY_MSG_SHUTDOWN);
    }

    // end serial connection to be able to detect power off
    swserial.end();

    shutdown_timestamp = timestamp + WAIT_TIMEOUT_MS;
    power_status = WAIT_FOR_SHUTDOWN;

  } else if (power_status == WAIT_FOR_SHUTDOWN) {

    // check if the pi has been shutdown completely
    if (check_shutdown()) {
      // shutdown completed: turn off and wait for power on
      power_off(true);
    } else if (timestamp >= shutdown_timestamp) {
      // time out waiting for poweroff changed event: power off anyway
      power_off(true);
    }

  } else if (power_status == HARD_SHUTDOWN) {

    // hard shutdown
    power_off(false);
  }

  // proceed to next iteration
}
