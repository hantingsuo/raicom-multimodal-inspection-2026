# Code map

The competition controller is intentionally released as the integrated source
used by the hardware system. This guide provides a stable navigation layer
without refactoring tested control paths before publication.

## Shared platform layer

- robot-state, IMU, foot-force, and LiDAR subscribers;
- front serial range-sensor reader;
- camera-opening and exposure-control helpers;
- angle normalization, timing, stopping, and yaw-baseline utilities.

## Visual perception

- `classifyWarningAction` — recognizes the warning-action command;
- `classifyPlaceCommand` — recognizes the placement command;
- `recognizeWarningFromCamera` and `recognizePlaceFromCamera` — aggregate
  camera observations before the mission commits to a command;
- line and region segmentation helpers — provide adaptive thresholds and
  failure diagnostics for course following.

## Go2 motion behaviors

- `turnInPlace` and `turnToYawDeg` — bounded yaw maneuvers;
- `alignFinalLineCenter` — final visual alignment;
- `runLineFollowing` — adaptive line-following state machine;
- `runAvoidance` — staged obstacle-avoidance sequence;
- `runStairs` — guarded ascent, summit, turn, descent, and exit phases;
- `runArcToPlatform`, `runThreeTurnDetect`, and `runUntilDualPlatform` —
  approach and platform-transition behaviors.

## External manipulator coordination

- `openArmSerial` — opens the external serial interface;
- `waitArmReply` — bounded acknowledgement and timeout handling;
- `sendArmLineAcked` and `sendSimpleArmCommand` — Go2-side command transport;
- `runMaterialGrab` and `runSecondMaterialGrab` — mission-level coordination
  of perception, robot positioning, and the external manipulator.

These functions reveal the public integration protocol from the Go2 side. The
firmware and internal implementation running on the manipulator are not part of
this repository.

## Entry point and field modes

`main` parses the network interface, course choices, calibration modes, camera
parameters, and serial-device overrides before initializing Unitree SDK2 and
running the integrated mission. Hardware calibration modes must be used while
the robot is stationary and under direct supervision.
