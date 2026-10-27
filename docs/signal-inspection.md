# Inspect saved signals

In **Review & export**, select `trial.json` and press **Plot / export image**. Choose the signals, optional spectrum and PNG destination. Leave the image destination empty to open an interactive plot. The plot shows
raw EMG, upper-arm and forearm acceleration/gyroscope readings, and saved camera
angle estimates. Use the toolbar to zoom, pan or save an image.

Select a CSV or `pose.json` to inspect one source. Plotting does not filter,
normalise or overwrite recordings. Camera angles are estimates, not a reference
measurement. IMU units come from recording metadata; missing units are labelled
as recorded units.

EMG and IMUs share the saved ESP32 clock. Camera alignment uses saved clock-sync
timestamps. If those timestamps are missing, the axes show their separate clock
bases. Operator lines mark button presses, not verified grasp/release times.
Gaps larger than 1.5 times the median device sample interval break the plotted line;
no missing samples are filled.

Before processing, check:

- The relaxed level and the change during a grip.
- Flat or clipped sections at the ADC limits.
- Spikes, missing samples and movement artefacts.
- Whether the cue/marker timing agrees with the signal and video.

Enable **Include EMG spectrum** in that form. The equivalent optional command is:

```powershell
python -m aurora.recording.plot_sensor_recording --file PATH/emg.csv --spectrum
```

This uses Welch's power spectral density estimate with a Hann window, up to
4096 samples per segment, 50% overlap and segment mean removal. It rejects gaps
instead of silently interpolating. The sampling rate comes from ESP32 acquisition
timestamps, so clock-sync corrections do not change the spectrum. Peaks can help identify periodic interference;
they do not establish signal quality or grip accuracy by themselves.

After inspecting the raw data, use **Process EMG**. The default offline pipeline
removes the offset, applies a fourth-order 20-450 Hz Butterworth band-pass and
a 50 Hz notch, then exports RMS, MAV, waveform length, zero crossings,
slope-sign changes and sample variance in 200 ms windows with 50 ms overlap.
The ZC/SSC threshold defaults to 5 ADC counts (`--noise-threshold`).
It requires regular sample times and a sampling rate above twice the high cutoff.
The USB diagnostic stream is 400 Hz, so it cannot use the default 450 Hz cutoff.
Choose a lower **Band-pass upper frequency** in the processing form, for example 150 Hz. The equivalent optional command is:

```powershell
python -m aurora.recording.emg_processing --file PATH/recording.csv --highcut 150
```

That is an explicit analysis choice, not the default for full-rate BLE data.
Filtered samples, features and settings are saved as separate files. Processing
metadata includes the source filename and SHA-256 hash. With `--labels`, the same
filter and window options apply, and the cue file hash is saved too. Keep the
originals. Offline forward-backward filtering uses future samples and differs
from the causal live filter. [EMG learning](emg-learning.md) uses a separate,
shared causal pipeline for training and live prediction.

The earlier HT25 `src/emg/emg_processing_utils.py` provides the offset removal,
band-pass/notch and RMS/MAV baseline. The earlier EMG3 report by Folkesson et al.
(2025), sections III-B and IV-B, compares preprocessing pipelines and uses 200 ms
windows. Its filters and hardware differ from this setup.
Welch's method is described by P. Welch (1967),
[doi:10.1109/TAU.1967.1161901](https://doi.org/10.1109/TAU.1967.1161901), and the
[SciPy documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.welch.html).
