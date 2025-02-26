/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/

#undef TAPPING_TERM
#define TAPPING_TERM 165

#undef RGB_MATRIX_TIMEOUT
#define RGB_MATRIX_TIMEOUT 120000

#define USB_SUSPEND_WAKEUP_DELAY 0
#define SERIAL_NUMBER "NXaXB/lbb39w"
#define LAYER_STATE_8BIT
#define COMBO_COUNT 1
#define HCS(report) host_consumer_send(record->event.pressed ? report : 0); return false

#define TAPPING_TERM_PER_KEY
#define RGB_MATRIX_STARTUP_SPD 60

