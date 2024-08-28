/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2024 Marek S. Łukasiewicz
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

#include <pico/stdio.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/board_api.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pio_usb.h"
#include "pio_usb_configuration.h"
#include "tusb.h"

#include "usb_definitions.h"
#include "usb_descriptors.h"

enum {
    BLINK_NO_DEVICE = 0,
    BLINK_NO_DATA = 2000,
    BLINK_REPORT = 500,
};

static uint32_t blink_interval_ms = BLINK_NO_DEVICE;

static usb_device_t *usb_device = NULL;

void led_blinking_task(void);
void receive_mouse_task(void);
void send_mouse_task(void);
void mouse_movement(int8_t *out_move_x, int8_t *out_move_y);
void handle_report(int len, uint8_t *data);

/*------------- MAIN -------------*/
void core1_main() {
    sleep_ms(10);

    // Connect mouse D+ to gp0 and D- to gp1
    static pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
    // To run USB SOF interrupt in core1, create alarm pool in core1
    config.alarm_pool = (void *)alarm_pool_create(2, 1);
    usb_device = pio_usb_host_init(&config);

    while (true) {
        pio_usb_host_task();
    }
}

int main() {
    // Sysclock should be multiple of 12MHz for USB
    set_sys_clock_khz(120000, true);

    board_init();

    multicore_reset_core1();
    // USB host tasks run in core1
    multicore_launch_core1(core1_main);

    // Init TinyUSB device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    while (true) {
        tud_task(); // tinyusb device task
        led_blinking_task();
        receive_mouse_task();
        send_mouse_task();
    }
}

int32_t mouse_pos_x = 0;
int32_t mouse_pos_y = 0;
int32_t target_pos_x = 0;
int32_t target_pos_y = 0;

const float time_constant_ms = 5000;
float last_movement_ms = 0;
void mouse_movement(int8_t *out_move_x, int8_t *out_move_y) {
    float time_ms = board_millis();

    // Only initialize on first call
    if (last_movement_ms == 0) {
        last_movement_ms = time_ms;
        out_move_x = 0;
        out_move_y = 0;
        return;
    }

    // First order filter (move to reach target after time constant)
    float delta_time_ms = time_ms - last_movement_ms;
    delta_time_ms = delta_time_ms <= 0 ? 0.1 : delta_time_ms;
    float move_fraction = time_constant_ms / delta_time_ms;
    move_fraction = move_fraction <= 0 ? 0.1 : move_fraction;

    float move_x = (target_pos_x - mouse_pos_x) / move_fraction;
    float move_y = (target_pos_y - mouse_pos_y) / move_fraction;
    // int32_t move_x = target_pos_x - mouse_pos_x;
    // int32_t move_y = target_pos_y - mouse_pos_y;
    *out_move_x = (move_x < -127 ? -127 : (move_x > 128 ? 127 : move_x));
    *out_move_y = (move_y < -127 ? -127 : (move_y > 128 ? 127 : move_y));

    // Update state
    mouse_pos_x += *out_move_x;
    mouse_pos_y += *out_move_y;
    last_movement_ms = time_ms;
}

//--------------------------------------------------------------------+
// Host logic
//--------------------------------------------------------------------+
void receive_mouse_task(void) {
    if (usb_device != NULL) {
        for (int dev_idx = 0; dev_idx < PIO_USB_DEVICE_CNT; dev_idx++) {
            usb_device_t *device = &usb_device[dev_idx];
            if (!device->connected) {
                continue;
            }

            // Print packets received to endpoints
            for (int ep_idx = 0; ep_idx < PIO_USB_DEV_EP_CNT; ep_idx++) {
                endpoint_t *ep = pio_usb_get_endpoint(device, ep_idx);
                if (ep == NULL) {
                    break;
                }

                uint8_t temp[64];
                int len = pio_usb_get_in_data(ep, temp, sizeof(temp));

                if (len > 0) {
                    blink_interval_ms = BLINK_REPORT;
                }
                handle_report(len, temp);
            }
        }
    }
}

uint8_t received_buttons = 0;
int8_t received_scroll = 0;
int8_t received_pan = 0;
void handle_report(int len, uint8_t *data) {
    if (len == 7 && data[0] == 0x02) {
        // Assume report like esperanza mouse:
        // ID       8 bits = 02
        // buttons  8 bits
        // move X   12 bits
        // move Y   12 bits
        // scroll   8 bits
        // pan      8 bits
        received_buttons = (uint8_t)data[1];

        // From Jippity and some forum:
        int16_t move_x = ((data[3] & 0x0F) << 8) | data[2];
        int16_t move_y = (data[4] << 4) | ((data[3] & 0xF0) >> 4);
        // sign extension
        if (move_x >= 0x800)
            move_x -= 0x1000;
        if (move_y >= 0x800)
            move_y -= 0x1000;

        // // Scale from 12 bit to 8 bit movement
        // move_x /= (2048 / 128);
        // move_y /= (2048 / 128);

        target_pos_x += move_x;
        target_pos_y += move_y;

        received_scroll = data[5];
        received_pan = data[6];
    }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
    // blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    // blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    // blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    // blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

bool output_active = false;
static void send_hid_report(uint32_t btn) {
    // skip if hid is not ready yet
    if (!tud_hid_ready())
        return;

    int8_t delta_x = 0, delta_y = 0;
    mouse_movement(&delta_x, &delta_y);

    tud_hid_mouse_report(REPORT_ID_MOUSE, received_buttons, delta_x, delta_y,
                         received_scroll, received_pan);
    received_scroll = 0;
    received_pan = 0;
}

// Every 10ms, send mouse HID report
void send_mouse_task(void) {
    // Poll every 10ms
    const uint32_t interval_ms = 10;
    static uint32_t start_ms = 0;

    if (board_millis() - start_ms < interval_ms)
        return; // not enough time
    start_ms += interval_ms;

    uint32_t const btn = board_button_read();

    // Remote wakeup
    if (tud_suspended() && btn) {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        tud_remote_wakeup();
    } else {
        // Send the report
        send_hid_report(btn);
    }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report,
                                uint16_t len) {
    (void)instance;
    (void)len;

    uint8_t next_report_id = report[0] + 1u;

    // if (next_report_id < REPORT_ID_COUNT) {
    //     send_hid_report(board_button_read());
    // }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
    // TODO not Implemented
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
    (void)instance;

    if (report_type == HID_REPORT_TYPE_OUTPUT) {
        // Set keyboard LED e.g Capslock, Numlock etc...
        if (report_id == REPORT_ID_KEYBOARD) {
            // bufsize should be (at least) 1
            if (bufsize < 1)
                return;

            uint8_t const kbd_leds = buffer[0];

            // if (kbd_leds & KEYBOARD_LED_CAPSLOCK) {
            //     // Capslock On: disable blink, turn led on
            //     blink_interval_ms = 0;
            //     board_led_write(true);
            // } else {
            //     // Caplocks Off: back to normal blink
            //     board_led_write(false);
            //     blink_interval_ms = BLINK_MOUNTED;
            // }
        }
    }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
    static uint32_t start_ms = 0;
    static bool led_state = false;

    // blink is disabled
    if (!blink_interval_ms)
        return;

    // Blink every interval ms
    if (board_millis() - start_ms < blink_interval_ms)
        return; // not enough time
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}
