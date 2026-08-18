# Hardware setup

## Expected platform

- Unitree Go2 with a computer capable of running Unitree SDK2;
- camera devices accessible through OpenCV/V4L2;
- Go2 state and range topics required by the controller;
- front serial distance sensor;
- optional external manipulator controller connected through a serial device.

## Dependency setup

Install Unitree SDK2 from its official repository and install OpenCV 4 through
the system package manager or another trusted source. Do not copy vendor source
trees into this repository.

## Device selection

Linux camera and serial enumeration can change after reconnecting hardware.
Prefer stable `/dev/v4l/by-id/` and `/dev/serial/by-id/` paths where available.
Do not commit machine-specific aliases, serial numbers, Wi-Fi credentials, or
private calibration files.

## Calibration

Calibrate perception modules in a stationary mode before allowing locomotion.
Validate exposure, region-of-interest settings, range thresholds, and coordinate
transforms in the actual venue lighting and sensor arrangement. Local calibration
outputs are deliberately ignored by Git.
