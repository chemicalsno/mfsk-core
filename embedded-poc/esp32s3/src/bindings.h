// Extra bindgen header for the esp-idf-sys build. The wrapper symbol
// `ESP_IDF_COMP_ESPRESSIF__ESP_DSP_ENABLED` is defined automatically
// by ESP-IDF's component manager when esp-dsp is included as a managed
// component (see `[[package.metadata.esp-idf-sys.extra_components]]`
// in Cargo.toml).
#if defined(ESP_IDF_COMP_ESPRESSIF__ESP_DSP_ENABLED)
#include "esp_dsp.h"
#endif
