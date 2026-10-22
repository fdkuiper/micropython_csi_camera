#include <assert.h>
#include <string.h>
#include "../monochrome.h"

int main(void)
{
    // Two rows verify neutral U and V, luma endpoints and intermediate values.
    const uint8_t raw[] = {0, 255, 64, 128, 32, 192, 16, 240};
    uint8_t packed[14];
    memset(packed, 0xa5, sizeof(packed));
    camera_mono_convert(raw, packed + 1, 8, true);
    const uint8_t expected[] = {128, 16, 235, 128, 71, 126,
                                128, 43, 181, 128, 30, 222};
    assert(memcmp(packed + 1, expected, sizeof(expected)) == 0);
    assert(packed[0] == 0xa5 && packed[13] == 0xa5);
    uint8_t rgb[6];
    const uint8_t ramp[] = {0, 128, 255};
    camera_mono_convert(ramp, rgb, 3, false);
    const uint8_t rgb_expected[] = {0, 0, 0x10, 0x84, 255, 255};
    assert(memcmp(rgb, rgb_expected, sizeof(rgb)) == 0);
    return 0;
}
