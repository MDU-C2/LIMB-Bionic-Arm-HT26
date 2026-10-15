# EMG calibration signal check — 12 October 2026

The ESP32 was powered by a separate USB source. All sensor data was read over
BLE. Two coached six-second captures compared relaxed rest and a requested
strong grip. Their raw data and summaries are saved under ignored `outputs/`.

| Check | Measured result |
| --- | --- |
| EMG rate | 4,000 samples/s |
| Initial rest/grip packet health | Zero missing packets, sequence gaps or decode errors |
| Rest filtered RMS | 2.49 ADC counts offline; 2.47 median online |
| Requested grip filtered RMS | 2.69 ADC counts offline; 2.70 median online |
| Raw signal power below 5 Hz | 98.67% rest; 98.32% requested grip |
| ADC clipping | None in either capture |
| IMUs | Both reporting; acceleration near 1 g, low stationary angular velocities |
| OAK-D Lite RGB | 12/12 frames at 640×480 |

The requested grip signal rose only about 8% above rest. These recordings do not
establish a reliable grip-related contraction. Large raw ADC swings were mostly
slow drift, so treating their amplitude as grip would also respond to drift.

The ADC reads GPIO0 directly and does not rectify the samples. Electrode
contact and the amplifier gain still need checking on the connected setup.

Software changes:

- Exclude the filter's settling transient from the rest noise floor.
- Replace the old five-times-noise/fixed-five-count calibration requirement
  with a gain-relative noise margin, while retaining a minimum one-count margin.
- Use the sustained median grip level, preventing a short spike from setting
  full grip.
- Show current RMS, rest level, measured grip and the required level.
- Interrupt grip calibration on missing BLE samples and suppress filter
  transients after a discontinuity. Duplicate/reordered samples are skipped.

A subsequent simultaneous camera check had intermittent BLE loss (52 missing
notifications in each stream); the original rest/grip captures had none.
The new discontinuity guard prevents that loss from becoming false EMG activity.
Synthetic checks cover low-amplitude but distinct contractions, transient
settling, spikes, clipping, repeatability and packet gaps. Physical electrode
contact, reference connection and analogue gain still need checking when the
reported grip level stays close to rest.

Diagnostic commands, using the project's managed interpreter:

```powershell
python scripts/check_emg_calibration.py --phase rest --seconds 6
python scripts/check_emg_calibration.py --phase grip --interactive --seconds 6
```

Close AURORA before this direct diagnostic so the board can advertise. It opens
no serial port, flashes no firmware, and commands no physical actuator.
