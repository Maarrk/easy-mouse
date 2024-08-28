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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void hid_task(void);
void mouse_movement(int8_t *out_move_x, int8_t *out_move_y);

/*------------- MAIN -------------*/
int main(void) {
    board_init();

    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    while (1) {
        tud_task(); // tinyusb device task
        led_blinking_task();

        hid_task();
    }
}

int32_t mouse_pos_x = 0;
int32_t mouse_pos_y = 0;
int32_t target_pos_x = 0;
int32_t target_pos_y = 0;

const int32_t time_constant_ms = 500;
uint32_t last_movement_ms = 0;
void mouse_movement(int8_t *out_move_x, int8_t *out_move_y) {
    uint32_t time_ms = board_millis();

    // Only initialize on first call
    if (last_movement_ms == 0) {
        last_movement_ms = time_ms;
        out_move_x = 0;
        out_move_y = 0;
        return;
    }

    // Move target every 2 seconds
    uint8_t phase = (time_ms / 2000) % 4;
    switch (phase) {
    case 0:
        target_pos_x = 200;
        target_pos_y = -200;
        break;

    case 1:
        target_pos_x = 200;
        target_pos_y = 200;
        break;

    case 2:
        target_pos_x = -200;
        target_pos_y = 200;
        break;

    case 3:
        target_pos_x = -200;
        target_pos_y = -200;
        break;

    default:
        target_pos_x = 0;
        target_pos_y = 0;
        break;
    }

    // First order filter (move to reach target after time constant)
    int32_t delta_time_ms = time_ms - last_movement_ms;
    if (delta_time_ms == 0) delta_time_ms = 1; // prevent division by 0
    int32_t move_fraction = time_constant_ms / delta_time_ms;
    if (move_fraction == 0) move_fraction = 1; // prevent division by 0

    *out_move_x = (target_pos_x - mouse_pos_x) / move_fraction;
    *out_move_y = (target_pos_y - mouse_pos_y) / move_fraction;
    
    // Update state
    mouse_pos_x += *out_move_x;
    mouse_pos_y += *out_move_y;
    last_movement_ms = time_ms;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) { blink_interval_ms = BLINK_MOUNTED; }

// Invoked when device is unmounted
void tud_umount_cb(void) { blink_interval_ms = BLINK_NOT_MOUNTED; }

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

bool output_active = false;
static void send_hid_report(uint32_t btn) {
    // skip if hid is not ready yet
    if (!tud_hid_ready())
        return;

    if (btn)
        output_active = true;

    int8_t delta_x = 0, delta_y = 0;
    if (output_active) {
        mouse_movement(&delta_x, &delta_y);
    }

    tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta_x, delta_y, 0, 0);
}

// Every 10ms, send mouse HID report
void hid_task(void) {
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

            if (kbd_leds & KEYBOARD_LED_CAPSLOCK) {
                // Capslock On: disable blink, turn led on
                blink_interval_ms = 0;
                board_led_write(true);
            } else {
                // Caplocks Off: back to normal blink
                board_led_write(false);
                blink_interval_ms = BLINK_MOUNTED;
            }
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
