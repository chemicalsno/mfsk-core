// Extra bindgen header for the esp-idf-sys build. Identical to the
// ESP32-S3 PoC — esp-dsp is the same managed component on both chips.
#if defined(ESP_IDF_COMP_ESPRESSIF__ESP_DSP_ENABLED)
#include "esp_dsp.h"
#endif
