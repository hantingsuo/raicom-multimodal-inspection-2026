# Third-party notices

The repository contains the project's high-level integration code. It does not
vendor the following dependencies or interoperability targets.

| Project | Use | Upstream license | Distribution decision |
|---|---|---|---|
| [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) | Go2 communication and motion API | BSD 3-Clause | Install from upstream; source and binaries are not copied here. |
| [OpenCV](https://opencv.org/) | Camera capture and computer vision | Apache 2.0 | Install separately; source and binaries are not copied here. |
| [Waveshare RoArm-M2](https://github.com/waveshareteam/roarm_m2) | External manipulator interoperability | GNU GPL v3 or later in the upstream repository | Firmware and vendor examples are not redistributed here. |

The repository's BSD 3-Clause License applies only to original project code and
documentation. Each external dependency and hardware component remains governed
by its own license and terms.
