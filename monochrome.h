#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// RAW8 has one luminance sample per pixel, with no Bayer pattern.
// Packed YUV420 alternates UYY and VYY lines; neutral chroma is 128.
static inline void camera_mono_convert(const uint8_t *raw, uint8_t *out,
                                       size_t pixels, bool h264)
{
    if (h264) {
        for (size_t i = 0; i < pixels; i += 2) {
            *out++ = 128;
            *out++ = 16 + ((unsigned)raw[i] * 219 + 127) / 255;
            *out++ = 16 + ((unsigned)raw[i + 1] * 219 + 127) / 255;
        }
    } else {
        for (size_t i = 0; i < pixels; ++i) {
            uint16_t rgb = ((raw[i] >> 3) << 11) |
                           ((raw[i] >> 2) << 5) | (raw[i] >> 3);
            *out++ = rgb & 255;
            *out++ = rgb >> 8;
        }
    }
}
