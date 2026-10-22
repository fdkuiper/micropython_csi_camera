Because CSI does not have the function of automatic camera model identification, it is necessary to add the setting model and picture size to its own board, such as:

```
CONFIG_CAMERA_OV5647=y
CONFIG_CAMERA_OV5647_MIPI_RAW8_800x640_50FPS=y
```

Then modify the config.h file according to the set size, and refer to the official C-bound MicroPython document when compiling it into firmware.



## OV9281 selection

The FireBeetle board fork sets `MICROPY_CAMERA_OV9281 ON` before including this
module. Both sensor drivers are available in one firmware image. OV5647 remains
the default, including existing positional arguments and default dimensions.

```python
camera.init(sensor='OV9281', w=1280, h=720, h264=True, fps=20, gop=5)
frame, is_idr = camera.capture()
print(camera.formats())
camera.deinit()
```

OV9281 also supports 640x400 RAW8. Specify dimensions when selecting it.
Selecting a sensor verifies that sensor's chip ID; unsupported sensor names,
dimensions or frame rates raise exceptions. Deinitialize before changing modes.
JPEG capture is also supported with `h264=False`.

See [backport details](sensors/ov9281/README.md) for provenance, clock and capture
requirements. Board connector/power compatibility and real camera operation must
be verified on hardware. Host conversion check:

```sh
cc -Wall -Wextra -Werror tests/test_monochrome.c -o /tmp/test_monochrome
/tmp/test_monochrome
```
