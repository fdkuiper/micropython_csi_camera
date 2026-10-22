# OV9281 backport

Apache-2.0 sources from Espressif esp-video-components commit
`3ae2756e21c5f85ed8d9809100ab02d079347a79`, directory
`esp_cam_sensor/sensors/ov9281`:
https://github.com/espressif/esp-video-components/tree/3ae2756e21c5f85ed8d9809100ab02d079347a79/esp_cam_sensor/sensors/ov9281

Changes for the pinned esp_cam_sensor 1.7.0 and ESP-IDF 5.5 build:
- Build both RAW8 modes (1280x720 at 50 fps, 640x400 at 100 fps).
- Replace Kconfig-only gain/default-format settings with their upstream defaults;
  remove auto-detection registration. The wrapper probes explicit 7-bit addresses
  0x60, 0x70 and 0x10 and verifies chip ID 0x9281.
- Add explicit standard/heap includes; no generated/managed component edits.
- Describe lane rate as 800 Mbps (400 MHz DDR clock), matching the register tables.
- Wait 5 ms after software reset; leave mode tables in standby until the wrapper
  starts CSI capture. The module must supply its own 24 MHz input clock.

This is a monochrome sensor. The wrapper uses ISP bypass with RAW8 DMA and
converts pixels to RGB565 for JPEG or packed UYY/VYY with neutral chroma and
limited-range luma for hardware H.264. No Bayer demosaicing is applied.
There is no automatic exposure loop; the sensor uses the register-table defaults.
Hardware capture, brightness and sustained stream rates still need validation.
