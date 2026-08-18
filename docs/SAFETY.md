# Safety

This repository controls physical robot motion. Incorrect parameters, sensor
failures, or unexpected environment conditions can cause falls, collisions,
equipment damage, or personal injury.

Before running the integrated mission:

1. test every sensing and control stage independently;
2. verify the network interface, cameras, range sensors, and serial devices;
3. clear the operating area and keep people outside the motion envelope;
4. keep the robot's emergency stop immediately accessible;
5. begin with conservative speed limits and a supported test surface;
6. do not test stairs without spotters and a fall-mitigation plan;
7. stop immediately after stale, missing, or inconsistent sensor readings.

The included defaults describe one competition setup and are not safety
guarantees for another robot or environment.
