/**
 * Raspberry Pi Pico USB HID Touchscreen Simulator - Draws a circle continuously
 * 
 * Based on U2HTS project structure. Generates a single contact moving in a
 * circular path around screen center and sends HID reports.
 */

#include <math.h>
#include <string.h>

#include "bsp/board_api.h"
#include "pico/stdlib.h"
#include "tusb.h"

// -------------------- HID Report Definitions --------------------
#define U2HTS_HID_TP_REPORT_ID       1
#define U2HTS_HID_TP_MAX_COUNT_ID    2

// Logical and physical maximum coordinates (defines screen resolution)
#define U2HTS_LOGICAL_MAX            2048
#define U2HTS_PHYSICAL_MAX_X         2048
#define U2HTS_PHYSICAL_MAX_Y         2048

// Number of simulated touch points (we use only one)
#define MAX_TOUCH_POINTS             5

// Macro to build HID descriptor for one touch point
#define U2HTS_HID_TP_DESC \
  HID_USAGE(0x22), HID_COLLECTION(HID_COLLECTION_LOGICAL), \
  HID_USAGE(0x42), HID_LOGICAL_MAX(1), HID_REPORT_SIZE(1), HID_REPORT_COUNT(1), \
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_REPORT_COUNT(7), HID_INPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE), \
  HID_REPORT_SIZE(8), HID_USAGE(0x51), HID_REPORT_COUNT(1), \
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP), \
  HID_LOGICAL_MAX_N(U2HTS_LOGICAL_MAX, 2), HID_REPORT_SIZE(16), \
  HID_USAGE(HID_USAGE_DESKTOP_X), \
  HID_PHYSICAL_MAX_N(U2HTS_PHYSICAL_MAX_X, 2), \
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_PHYSICAL_MAX_N(U2HTS_PHYSICAL_MAX_Y, 2), \
  HID_USAGE(HID_USAGE_DESKTOP_Y), \
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DIGITIZER), \
  HID_LOGICAL_MAX_N(255, 2), HID_PHYSICAL_MAX_N(255, 2), \
  HID_REPORT_SIZE(8), HID_REPORT_COUNT(3), \
  HID_USAGE(0x48), HID_USAGE(0x49), HID_USAGE(0x30), \
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_COLLECTION_END

// Descriptor for scan time and contact count
#define U2HTS_HID_TP_INFO_DESC \
  HID_LOGICAL_MAX_N(0xFFFF, 3), HID_REPORT_SIZE(16), HID_UNIT_EXPONENT(0x0C), \
  HID_UNIT_N(0x1001, 2), HID_REPORT_COUNT(1), HID_USAGE(0x56), \
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_USAGE(0x54), HID_LOGICAL_MAX(MAX_TOUCH_POINTS), HID_REPORT_SIZE(8), \
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE)

// Feature report for maximum contacts
#define U2HTS_HID_TP_MAX_COUNT_DESC \
  HID_USAGE(0x55), HID_FEATURE(HID_DATA | HID_VARIABLE | HID_ABSOLUTE)

// -------------------- Data Structures --------------------
typedef struct __packed {
  bool    contact;   // Tip switch (1 = touch present)
  uint8_t id;        // Contact identifier
  uint16_t x;        // X coordinate
  uint16_t y;        // Y coordinate
  uint8_t width;     // Contact width
  uint8_t height;    // Contact height
  uint8_t pressure;  // Contact pressure
} u2hts_tp;

typedef struct __packed {
  u2hts_tp tp[MAX_TOUCH_POINTS];
  uint16_t scan_time;   // Scan time in 0.1ms units
  uint8_t  tp_count;    // Number of valid contacts in this report
} u2hts_hid_report;

// -------------------- USB Descriptors --------------------
static const tusb_desc_device_t device_desc = {
    .bLength            = sizeof(device_desc),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x2E8A,   // Raspberry Pi
    .idProduct          = 0x8572,   // 'UH' in ASCII
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

static const uint8_t hid_report_desc[] = {
    HID_USAGE_PAGE(HID_USAGE_PAGE_DIGITIZER),
    HID_USAGE(0x04),                         // Touch screen
    HID_COLLECTION(HID_COLLECTION_APPLICATION),

    HID_REPORT_ID(U2HTS_HID_TP_REPORT_ID),
    HID_USAGE(0x22),                         // Finger
    HID_PHYSICAL_MIN(0),
    HID_LOGICAL_MIN(0),
    HID_UNIT_EXPONENT(0x0E),
    HID_UNIT(0x11),

    // Repeat for each possible contact (5 points)
    U2HTS_HID_TP_DESC,
    U2HTS_HID_TP_DESC,
    U2HTS_HID_TP_DESC,
    U2HTS_HID_TP_DESC,
    U2HTS_HID_TP_DESC,

    U2HTS_HID_TP_INFO_DESC,

    HID_REPORT_ID(U2HTS_HID_TP_MAX_COUNT_ID),
    U2HTS_HID_TP_MAX_COUNT_DESC,

    HID_COLLECTION_END
};

static const uint8_t config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_desc),
                       0x81, CFG_TUD_HID_EP_BUFSIZE, 5)
};

static uint16_t _desc_str[32 + 1];
static const uint8_t *string_desc_arr[] = {
    (const uint8_t[]){0x09, 0x04},          // Language: English (US)
    (const uint8_t *)"U2HTS",
    (const uint8_t *)"USB HID Touch Simulator",
    NULL                                     // Serial number filled later
};

// -------------------- TinyUSB Callbacks --------------------
uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&device_desc;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return hid_report_desc;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return config_desc;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;

    switch (index) {
        case 0:
            memcpy(&_desc_str[1], string_desc_arr[0], 2);
            chr_count = 1;
            break;
        case 3:
            chr_count = board_usb_get_serial(_desc_str + 1, 32);
            break;
        default:
            if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
                return NULL;
            const uint8_t *str = string_desc_arr[index];
            chr_count = strlen((const char *)str);
            if (chr_count > 31) chr_count = 31;
            for (size_t i = 0; i < chr_count; i++)
                _desc_str[1 + i] = str[i];
            break;
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

// -------------------- HID Report Handlers --------------------
static bool host_ready = false;   // Set after host requests max contacts

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)bufsize;
    // Not used
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
    (void)instance;
    if (report_type == HID_REPORT_TYPE_FEATURE &&
        report_id == U2HTS_HID_TP_MAX_COUNT_ID) {
        buffer[0] = MAX_TOUCH_POINTS;
        host_ready = true;   // Host is ready to receive touch reports
        return 1;
    }
    return 0;
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report,
                                uint16_t len) {
    (void)instance; (void)report; (void)len;
    // Report sent, ready for next
}

// -------------------- Circle Drawing Logic --------------------
#define CIRCLE_CENTER_X   (U2HTS_PHYSICAL_MAX_X / 2)
#define CIRCLE_CENTER_Y   (U2HTS_PHYSICAL_MAX_Y / 2)
#define CIRCLE_RADIUS     500
#define STEP_ANGLE        0.1f      // radians per step

static float angle = 0.0f;

// Fill a HID report with a single touch at current angle
static void prepare_circle_report(u2hts_hid_report *report) {
    memset(report, 0, sizeof(*report));

    // Compute coordinates on circle
    uint16_t x = (uint16_t)(CIRCLE_CENTER_X + CIRCLE_RADIUS * cosf(angle));
    uint16_t y = (uint16_t)(CIRCLE_CENTER_Y + CIRCLE_RADIUS * sinf(angle));

    // Clamp to valid range
    if (x > U2HTS_PHYSICAL_MAX_X) x = U2HTS_PHYSICAL_MAX_X;
    if (y > U2HTS_PHYSICAL_MAX_Y) y = U2HTS_PHYSICAL_MAX_Y;

    // Fill first contact
    report->tp[0].contact  = true;
    report->tp[0].id       = 1;
    report->tp[0].x        = x;
    report->tp[0].y        = y;
    report->tp[0].width    = 30;
    report->tp[0].height   = 30;
    report->tp[0].pressure = 30;

    report->tp_count = 1;
    report->scan_time = (uint16_t)(to_us_since_boot(time_us_64()) / 100); // 0.1ms units
}

// -------------------- Main --------------------
int main(void) {
    stdio_init_all();
    board_init();
    tud_init(BOARD_TUD_RHPORT);

    // Wait for USB to be connected and configured
    while (!tud_mounted()) {
        tud_task();
        sleep_ms(10);
    }

    // Wait for host to request max contacts (sets host_ready)
    while (!host_ready) {
        tud_task();
        sleep_ms(10);
    }

    // Main loop: send touch reports to draw a circle
    while (1) {
        tud_task();

        u2hts_hid_report report;
        prepare_circle_report(&report);

        // Send report if USB is ready and not busy
        if (tud_hid_ready()) {
            tud_hid_report(U2HTS_HID_TP_REPORT_ID, &report, sizeof(report));
        }

        // Advance angle for next point
        angle += STEP_ANGLE;
        if (angle > 2.0f * M_PI) {
            angle -= 2.0f * M_PI;
        }

        // Delay to control drawing speed (~10ms per point gives smooth motion)
        sleep_ms(10);
    }

    return 0;
}
