/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "Capture.pio.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "remoteDescriptors.h"

#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
#define captureSize 432
#define capturePin 13
#define vS 14

void dmaHandler();
static inline void setupDma(PIO, uint);
static inline void setupPIO(PIO, uint, uint, uint, bool, float);

uint32_t buffer[captureSize];
uint dmaChan;
PIO pio;
uint sm;
uint8_t mouseState[5] = {0};
bool frozenState[5] = {0};
const uint32_t timeOut = 100; //25 ms after updates before clearing
uint32_t timeOutCounter = 0;


/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void hid_task(void);




/*------------- MAIN -------------*/
int main(void)
{
  stdio_init_all();
  dmaChan = dma_claim_unused_channel(true);
  float clkdiv = 125000000 / (38000 * 5);

  irq_set_exclusive_handler(DMA_IRQ_0, dmaHandler);
  irq_set_enabled(DMA_IRQ_0, true);
  printf("%d\n", irq_is_enabled(DMA_IRQ_0));

  pio = pio0;
  sm = pio_claim_unused_sm(pio, true);

  // uint capturePin = 13;
  // uint vS = 14;

  uint offset = pio_add_program(pio, &capture_program);

  //setup power to sensor
  gpio_init(vS);
  gpio_set_dir(vS, true);
  gpio_put(vS, true);
  //for interfacing with sensor
  gpio_pull_up(capturePin);

  setupDma(pio, sm);
  setupPIO(pio, sm, offset, capturePin, false, clkdiv);
    
  board_init();
  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();

    hid_task();
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn)
{
  // skip if hid is not ready yet
  if ( !tud_hid_ready() ) return;

  switch(report_id)
  {
    case REPORT_ID_MOUSE:
    {
      int8_t const delta = 5;

      // no button, right + down, no scroll, no pan
      if ((board_millis() - timeOutCounter) >= timeOut)
      {
          for (int i = 0; i < 5; i++)
          {
            if (true)
            {
              mouseState[i] = 0;
            }
          }
      }
      tud_hid_mouse_report(REPORT_ID_MOUSE, mouseState[0], mouseState[1], mouseState[2], mouseState[3], mouseState[4]);
      // if (btn)
      // {
      // tud_hid_mouse_report(REPORT_ID_MOUSE, 0x01, 0, 0, 0, 0);
      // }
    }
    break;
    default: break;
  }
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  uint32_t const btn = board_button_read();

  // Remote wakeup
  if ( tud_suspended() && btn )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }else
  {
    // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_MOUSE, btn);
  }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint8_t len)
{
  (void) instance;
  (void) len;

  uint8_t next_report_id = report[0] + 1;

  if (next_report_id < REPORT_ID_COUNT)
  {
    send_hid_report(next_report_id, board_button_read());
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance; //implementation unneeded?
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms) return;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  printf("test\n");
  led_state = 1 - led_state; // toggle
}

void dmaHandler()
{
    // printf("Received, proccessing ready\n");
    bool state = true;
    bool currentState;
    uint32_t stateWidth = 0;
    uint32_t tempWidth;
    uint32_t extractedData = 0;
    for (int i = 0; i < captureSize; i++)
    {
        for (int j = 0; j < 32; j++) //iterate from lsb of data
        {
            currentState = !((bool) ((buffer[i] >> j) && 1u)); //1 is off 0 is on
            if (currentState == state)
            {
                stateWidth += 1;
            }
            if (currentState != state)
            {
                // printf("State: %s, Width: %d\n", state ? "True": "False", stateWidth);
                if (state)
                {
                    tempWidth = stateWidth; // the on time divided by off time will determine if the signal is on or off
                }
                if (!state) //it is false so divide true by false
                {
                    float temp = (float) tempWidth / stateWidth;
                    // printf("%2.3f\n", temp);
                    if (temp < 1.2)
                    {
                        extractedData *= 2;
                        if (temp < 0.5)
                        {
                            extractedData += 1;
                        }
                    }

                }

                stateWidth = 1;
                state = !state;
            }
        }
    }
    if (stateWidth != 1)
    {
        // printf("State: %s, Width: %d\n", state ? "True": "False", stateWidth);
    }
    // printf("Transfer done\n");

    char n2 = extractedData & (char) 255;
    char n1 = (extractedData >> 8) & (char) 255;
    if (n1 + n2 == 255)
    {
        // printf("%u\n", extractedData);
        char id = n1 >> 1;
        char repeatBit = n1 & 1u;
        for (int i = 0; i < totalButtons; i++)
        {
          if (id == commandIDs[i])
          {
            for (int j = 0; j < 5; j++)
            {
              mouseState[j] = allDescriptors[i][j];
              timeOutCounter = board_millis();
            }
            break;
          }
        }
        printf("Raw: %u, Command: %u, Repeat: %d\n", extractedData, n1 >> 1, n1 & 1u);
    }


    dma_channel_acknowledge_irq0(dmaChan);
    pio_sm_exec(pio, sm, pio_encode_wait_pin(false, 0));
    dma_channel_set_write_addr(dmaChan, &buffer, true);
    pio_sm_clear_fifos(pio, sm);
    return;
}
static inline void setupPIO(PIO pio, uint sm, uint offset, uint pin, bool trigger, float clkdiv)
{
    pio_sm_config c = capture_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_in_shift(&c, true, true, 32);  //set autopush to true
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_exec(pio, sm, pio_encode_wait_pin(trigger, 0));
    pio_sm_set_enabled(pio, sm, true);

}

static inline void setupDma(PIO pio, uint sm)
{
    dma_channel_config dc = dma_channel_get_default_config(dmaChan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, false));
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);

    dma_channel_set_irq0_enabled(dmaChan, true);
    dma_channel_configure(dmaChan,
        &dc,
        &buffer,
        &pio->rxf[sm],
        captureSize,
        true
    );

}