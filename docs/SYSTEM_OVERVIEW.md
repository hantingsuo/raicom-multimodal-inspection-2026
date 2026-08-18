# System overview

## Public software boundary

The released C++ program runs on the Go2-side computer and owns the mission
state machine. It receives robot state and sensor observations, selects the
active navigation or inspection behavior, sends high-level locomotion commands
through Unitree SDK2, and exchanges compact serial commands with an external
manipulator controller.

The manipulator firmware is a separate executable component and is outside the
scope of this repository. The public source therefore documents the integration
boundary without redistributing the manipulator implementation.

## Main capability groups

### Perception

- camera acquisition through OpenCV;
- adaptive line and region segmentation;
- visual warning-sign recognition that selects action `1`, `2`, or `3`;
- visual placement-sign recognition that selects command `5` or `6`;
- range observations from the Go2 and the front distance sensor.

The two task decisions are recognition-only in the public revision. They cannot
be preselected on the command line. An inconclusive result leaves the runtime
identifier at zero and suppresses the corresponding action or serial command.

### Locomotion and navigation

- line following with loss detection and fallback behavior;
- staged obstacle avoidance;
- yaw-aware turns and platform alignment;
- stair ascent, summit handling, turning, and descent;
- controlled stopping and gait-transition guards.

### Mission coordination

- a single integrated execution flow;
- restart and calibration modes for field testing;
- timeouts and degraded-operation paths;
- serial coordination with the external manipulator subsystem.

## Hardware-specific parameters

Camera indexes, exposure values, serial-device paths, perception thresholds,
and motion timings are deployment parameters. Defaults in the source document
the competition configuration, but they must be revalidated before use on a
different robot, course, sensor layout, or lighting condition.
