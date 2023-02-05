/**
 * @file Pico-USB-Host-MIDI-Adapter.c
 * @brief A USB Host to Serial Port MIDI adapter that runs on a Raspberry Pi
 * Pico board
 *
 * MIT License

 * Copyright (c) 2022 rppicomidi

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "midi_uart_lib.h"
#include "bsp/board.h"
#include "tusb.h"
#include "class/midi/midi_host.h"
#include "pico/cyw43_arch.h"

// On-board LED mapping. If no LED, set to NO_LED_GPIO
//const uint NO_LED_GPIO = 255;
//const uint LED_GPIO = 25;
// UART selection Pin mapping. You can move these for your design if you want to
// Make sure all these values are consistent with your choice of midi_uart
// The default is to use UART 1, but you are free to use UART 0 if you make
// the changes in the CMakeLists.txt file or in your environment. Note
// that if you use UART0, then serial port debug will not be enabled
#ifndef MIDI_UART_NUM
#define MIDI_UART_NUM 1
#endif
#ifndef MIDI_UART_TX_GPIO
#define MIDI_UART_TX_GPIO 4
#endif
#ifndef MIDI_UART_RX_GPIO
#define MIDI_UART_RX_GPIO 5
#endif

static void *midi_uart_instance;
static uint8_t midi_dev_addr = 0;


/*************************/
/* multuplexer buttons */

#define MUL_BUTTONS 16
#define BTN_MUL_S0_PIN 18
#define BTN_MUL_S1_PIN 19
#define BTN_MUL_S2_PIN 20
#define BTN_MUL_S3_PIN 21
#define BTN_MUL_SIG_PIN 17

#define BTN_MUL_EN_PIN 16
float volumeParam = 1.0f;
float semiModifier = 0.5f;
bool noNote = 0;
bool fnMode = 0;
unsigned long loopCount = 0;
absolute_time_t startTime ;
uint8_t  keyMod = 40; //the range of value that can be added to the input note number to determine played note
typedef enum { IDLE,
               PRESSED,
               HOLD,
               RELEASED } KeyState;

typedef struct {
  uint8_t id;
  uint8_t value;
  KeyState state;
  uint8_t note;
} Key;

Key keys[] = {
  { 0, HIGH, IDLE, 27 },
  { 1, HIGH, IDLE, 28 },
  { 2, HIGH, IDLE, 29 },
  { 3, HIGH, IDLE, 30 },
  { 4, HIGH, IDLE, 31 },
  { 5, HIGH, IDLE, 32 },
  { 6, HIGH, IDLE, 33 },
  { 7, HIGH, IDLE, 34 },
  { 8, HIGH, IDLE, 35 },
  { 9, HIGH, IDLE, 36 },
  { 10, HIGH, IDLE, 37 },
  { 11, HIGH, IDLE, 38 },
  { 12, HIGH, IDLE, 39 },
  { 13, HIGH, IDLE, 40 },
  { 14, HIGH, IDLE, 41 },
  { 15, HIGH, IDLE, 42 }
};


void ButtonsMul_Init(void) {
  Serial.print("Init");
  //memset(lastSendVal, 0xFF, sizeof(lastSendVal));

  pinMode(BTN_MUL_SIG_PIN, INPUT);
  pinMode(BTN_MUL_EN_PIN, OUTPUT);

  digitalWrite(BTN_MUL_EN_PIN, LOW);

  pinMode(BTN_MUL_S0_PIN, OUTPUT);
  pinMode(BTN_MUL_S1_PIN, OUTPUT);
  pinMode(BTN_MUL_S2_PIN, OUTPUT);
  pinMode(BTN_MUL_S3_PIN, OUTPUT);
}

void ButtonMul_Process(void) {
  for (int i = 0; i < MUL_BUTTONS; i++) {
    digitalWrite(BTN_MUL_S0_PIN, ((i & (1 << 0)) > 0) ? HIGH : LOW);
    digitalWrite(BTN_MUL_S1_PIN, ((i & (1 << 1)) > 0) ? HIGH : LOW);
    digitalWrite(BTN_MUL_S2_PIN, ((i & (1 << 2)) > 0) ? HIGH : LOW);
    digitalWrite(BTN_MUL_S3_PIN, ((i & (1 << 3)) > 0) ? HIGH : LOW);
    // digitalWrite(BTN_MUL_S0_PIN, LOW);
    // digitalWrite(BTN_MUL_S1_PIN, LOW);
    // digitalWrite(BTN_MUL_S2_PIN, LOW);
    // digitalWrite(BTN_MUL_S3_PIN, LOW);

    // give some time for transition
    delay(1);

    int8_t newValue = HIGH;
    if (keys[i].value != (newValue = digitalRead(BTN_MUL_SIG_PIN)))  // Only find keys that have changed state.
    {
      keys[i].value = newValue;

      keys[i].state = (newValue == LOW ? PRESSED : RELEASED);

      if (i == 0) {

        if (keys[i].state == PRESSED) {
          fnMode = true;
          //miniScreenString(1, 1, "CMD", HIGH);
        } else {
          fnMode = false;
          //miniScreenString(1, 1, "     ", HIGH);
        }
      } else {
        //miniScreenString(1, 2, "BTN: " + String(i), HIGH);
        if (fnMode && keys[i].state == PRESSED) {
          Serial.printf("CMD %d, %d changed: %d. State: %d\n", i, keys[i].note, newValue, keys[i].state);
          keyToCommand(i);
          keys[i].state = IDLE;
        } else {

          Serial.printf("Note %d, %d changed: %d. State: %d\n", i, keys[i].note, newValue, keys[i].state);
          keyToNote(keys[i].note, i);
          keys[i].state = IDLE;
        }
      }


      noNote = 0;  // reset flag to enable notes - flag was set because of a modifier keyboard command
    }
  }
}

/* /multiplexer buttons */

static void blink_led(void)
{
    static absolute_time_t previous_timestamp = {0};

    static bool led_state = false;

    // This design has no on-board LED
    absolute_time_t now = get_absolute_time();

    int64_t diff = absolute_time_diff_us(previous_timestamp, now);
    if (diff > 1000000) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);

        led_state = !led_state;
        previous_timestamp = now;
    }
}

static void poll_usb_rx(bool connected)
{
    // device must be attached and have at least one endpoint ready to receive a message
    if (!connected || tuh_midih_get_num_rx_cables(midi_dev_addr) < 1)
    {
        return;
    }
    tuh_midi_read_poll(midi_dev_addr);
}


static void poll_midi_uart_rx(bool connected)
{
    uint8_t rx[48];
    // Pull any bytes received on the MIDI UART out of the receive buffer and
    // send them out via USB MIDI on virtual cable 0
    uint8_t nread = midi_uart_poll_rx_buffer(midi_uart_instance, rx, sizeof(rx));
    if (nread > 0 && connected && tuh_midih_get_num_tx_cables(midi_dev_addr) >= 1)
    {
        uint32_t nwritten = tuh_midi_stream_write(midi_dev_addr, 0,rx, nread);
        if (nwritten != nread) {
            TU_LOG1("Warning: Dropped %lu bytes receiving from UART MIDI In\r\n", nread - nwritten);
        }
    }
}

int main() {
    if (cyw43_arch_init()) {
        printf("WiFi init failed");
        return -1;
    }

    bi_decl(bi_program_description("Provide a USB host interface for Serial Port MIDI."));
//    bi_decl(bi_1pin_with_name(LED_GPIO, "On-board LED"));
    bi_decl(bi_2pins_with_names(MIDI_UART_TX_GPIO, "MIDI UART TX", MIDI_UART_RX_GPIO, "MIDI UART RX"));

    board_init();
    printf("Pico MIDI Host to MIDI UART Adapter\r\n");
    tusb_init();

    // Map the pins to functions
//    gpio_init(LED_GPIO);
//    gpio_set_dir(LED_GPIO, GPIO_OUT);
    midi_uart_instance = midi_uart_configure(MIDI_UART_NUM, MIDI_UART_TX_GPIO, MIDI_UART_RX_GPIO);
    printf("Configured MIDI UART %u for 31250 baud\r\n", MIDI_UART_NUM);
    printf("Core num: %d\r\n", get_core_num());
//    printf("gpio mask: %d\r\n", gpio_mask);
    while (1) {
      tuh_task();

      blink_led();
      bool connected = midi_dev_addr != 0 && tuh_midi_configured(midi_dev_addr);

      poll_midi_uart_rx(connected);
      if (connected)
        tuh_midi_stream_flush(midi_dev_addr);
      poll_usb_rx(connected);
      midi_uart_drain_tx_buffer(midi_uart_instance);
    }
}

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx)
{
  printf("MIDI device address = %u, IN endpoint %u has %u cables, OUT endpoint %u has %u cables\r\n",
      dev_addr, in_ep & 0xf, num_cables_rx, out_ep & 0xf, num_cables_tx);

  if (midi_dev_addr == 0) {
    // then no MIDI device is currently connected
    midi_dev_addr = dev_addr;
  }
  else {
    printf("A different USB MIDI Device is already connected.\r\nOnly one device at a time is supported in this program\r\nDevice is disabled\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  if (dev_addr == midi_dev_addr) {
    midi_dev_addr = 0;
    printf("MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  }
  else {
    printf("Unused MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  }
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
    if (midi_dev_addr == dev_addr)
    {
        if (num_packets != 0)
        {
            uint8_t cable_num;
            uint8_t buffer[48];
            while (1) {
                uint32_t bytes_read = tuh_midi_stream_read(dev_addr, &cable_num, buffer, sizeof(buffer));
                if (bytes_read == 0)
                    return;
                if (cable_num == 0) {
                    uint8_t npushed = midi_uart_write_tx_buffer(midi_uart_instance,buffer,bytes_read);
                    if (npushed != bytes_read) {
                        TU_LOG1("Warning: Dropped %lu bytes sending to UART MIDI Out\r\n", bytes_read - npushed);
                    }
                }
            }
        }
    }
}

void tuh_midi_tx_cb(uint8_t dev_addr)
{
    (void)dev_addr;
}
