# Firmware

No production firmware is committed yet. The GUI will detect ESP-IDF projects
by their top-level `CMakeLists.txt` and PlatformIO projects by `platformio.ini`.

Before migrating firmware, identify the controller currently installed in the
arm and record its board revision, pin map, CAN identifiers, sensors,
actuators, toolchain version, and recovery procedure. Keep one canonical
project per controller and document safe startup, communication loss, limit
handling, and emergency stop behavior beside the code.

Do not treat an old LIMB firmware target as current until it has been matched
to the physical wiring and tested without an attached load.
