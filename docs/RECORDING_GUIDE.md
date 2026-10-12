# Recording guide

Use BLE for EMG and IMUs. The OAK-D camera connects to the PC over USB.
The ESP32 can stay on USB for power while its sensor data travels over BLE.

## 1. Connect and check the live signals

1. Power the ESP32 and turn on the PC's Bluetooth. Connect the OAK-D if needed.
2. Open Start.bat. The GUI starts a BLE monitor and opens the live graphs.
3. Leave the BLE device name as LIMBServer. It is the name to search for,
   not a connection indicator. Change selects another advertised name; it
   does not rename the ESP32. Manual Windows pairing is not required.
4. Wait for the header to show 2/2 IMUs and EMG receiving. Move each IMU and
   check its acceleration and gyro traces. Previewing does not save files.
5. Attach the EMG electrodes using the project's wiring/setup instructions.
   Compare relaxed muscle with a contraction. Press **1. Calibrate EMG at rest**
   after attaching them, and relax for half a second. Then press
   **2. Calibrate strong grip (3 s)** and hold a strong comfortable grip for the
   countdown. Repeat in the simulator after launching live control. Unattached
   electrodes do not provide a muscle-signal check.
6. EMG starts with a 0–4095 ADC scale. Enable auto-scale for small signals,
   checking the axis values because it also enlarges noise. Check the packet
   counters for gaps, missing notifications and decode errors.
7. Use Open camera monitor to check framing. Show the chosen arm, hand and hips.
   Close that camera preview before recording so the recorder can use the camera.

If the graph window was hidden, use Connect BLE / show graphs in the GUI header.
The USB monitor is an optional diagnostic tool for firmware with USB streaming
enabled; the default BLE firmware does not send sensor samples to that monitor.

## 2. Set up a trial

1. Open Recording. Keep ESP32 sensors (Bluetooth) selected, with IMU and EMG
   checked. Keep OAK-D camera selected when collecting camera data too.
2. Leave ESP32 sensors (USB diagnostic) unchecked for normal data collection.
3. Set Subject (for example S01), Trial ID (for example rest_01 or clench_01),
   Test type, and Camera arm. Reuse the same Subject for that participant and
   use a new Trial ID for each trial. The trial name is metadata; it does not
   automatically label individual samples or ask for a movement sequence.
4. Start with Duration set to 30 seconds. Use 0 for manual stopping.
5. Check Output; its default is outputs/recordings. Put sensor positions,
   electrode placement and the intended movements in your experiment notes.

## 3. Record

1. Press Start selected sources. The GUI disconnects its BLE preview before
   the recorder connects. Camera and BLE recorders launch with shared session
   metadata. Watch Activity for Connected and the subscribed IMU/EMG streams;
   startup/scanning time is not the recorded trial duration.
2. Perform the planned trial after both selected sources have started.
3. Let the duration finish, or press Stop recording sources and confirm.
   Wait for the saved-session messages before opening the files or unplugging.
4. The standalone BLE graphs reconnect when all managed programs finish.
   These graphs are a live preview; they do not draw the saved recording.

Live simulation control uses the same BLE device. Finish recording before
starting control; each mode owns its connection. Its graphs open with the
simulator and do not collect a dataset.
For simulation with unattached electrodes, leave Simulation's Grip source on
Camera. Select EMG when ready to control grip from muscle activity. This setting
changes simulated grip control; it does not change what the recorder captures.
Live rotation also requires fixed IMU mounting and the half-second rest baseline
after L. Follow Simulation's mounting checks; moving loose boards is a reception
test, not a check of anatomical accuracy. Mounting-axis selections do not alter
the raw recorded coordinates.

## 4. Check every saved session

Open Recordings, refresh the list, or use Open output folder. A combined trial
creates separate BLE and camera folders with the same session prefix.

- BLE: emg.csv, imu.csv, packets.jsonl and meta.json.
- Camera: video.mp4, pose.json, frame_timing.json and meta.json.
- Check metadata for the subject/trial, requested and subscribed streams,
  duration, packet counts and packet_health. Review any gaps, missing packets
  or decode errors before using a trial for training.
- Full-rate BLE EMG should be about 4,000 samples/second with 250 microsecond
  device spacing. Each connected IMU should be about 100 samples/second.
  Notification counts are not EMG sample counts; each notification batches samples.
- Check both IMU roles and compare rest against contraction. Use Plot selected
  CSV to inspect the saved traces. Flat, clipped or noise-only EMG needs a
  setup check even when Bluetooth delivery is working.
- Check the camera actually tracked the subject. A saved video with no visible
  person does not provide useful arm-pose training data. Video frame count can
  include duplicates added to preserve playback timing.
- Current BLE recordings include synced_host_monotonic_ns and time-sync
  metadata. Keep these fields for cross-sensor alignment. Physical camera/sensor
  acquisition alignment still needs a separate check; launching together does
  not establish exact simultaneous sampling.

Keep raw recordings. Existing offline EMG processing removes DC, applies a
20–450 Hz band-pass and a 50 Hz notch, and writes RMS/MAV features separately:

    python src/recording/emg_processing.py --file outputs/recordings/SESSION/emg.csv

Run this from the project environment and replace SESSION with the BLE folder.
It requires regularly spaced samples; a recording with gaps needs a continuous
segment before filtering. Processing preserves the source CSV.

## If no sensors appear

- Read the BLE status in the GUI header and the graph window. Scanning or
  Reconnecting means the reader has not established a usable connection yet.
- Check ESP32 power, PC Bluetooth and the advertised name LIMBServer.
- Stop other recording/control programs that might already own the connection.
- Press Connect BLE / show graphs to reopen or retry the monitor.
- If the GUI reports a missing dependency, run Setup.bat and restart it.
- Optional scan from the project environment:

      python scripts/check_ble.py --name LIMBServer

The connected delivery checks and remaining amplifier/timing checks are in
docs/HARDWARE_VALIDATION.md. Wiring is described in docs/SENSOR_DATA.md.
