# EMG labels and models

## Review recordings

In **Recordings**, select `trial.json` or raw EMG and press **Review EMG labels**.
Check the subject/trial IDs. Select an interval on the plot, choose its phase,
then press **Add interval**. Check the saved video with the slider. Remove and
replace incorrect intervals; leave uncertain sections unlabelled.

The phases are rest (relaxed/open), grip (closing), hold (maintaining the grasp)
and release (opening). EMG onset can precede visible motion. Use one consistent
boundary rule across subjects. **Load cue suggestions** imports cues for checking;
cup markers are not EMG labels. Press **Save reviewed labels** when finished.

`emg.labels.json` saves intervals in seconds from the first EMG sample, IDs,
review status and the raw source hash. Drafts cannot train a model. Original
samples and cues stay unchanged. Video alignment needs synchronized EMG times
and camera metadata. New videos use `frame_times.jsonl` to seek the last frame
received at or before the selected sample, including repeated encoded frames.
These are host receipt times, not exposure times. Older videos fall back to
video playback time and show that limitation.

## Compare classifiers

Press **Compare EMG models** and choose reviewed recordings from at least two
subjects, or run from the project environment:

```powershell
python -m aurora.ml.train_emg --data outputs/recordings
```

Defaults compare LDA, ANN and LSTM with RMS/MAV and all six features. New runs
go in `outputs/training/` with `training.json`, participant splits, raw/label
hashes, code hashes, package versions, scores and model bundles. Install the
optional `requirements-training.txt` packages for PyTorch/ONNX training;
live inference uses NumPy or ONNX Runtime.
Successful runs use status `complete`. Older `completed` reports remain readable.
Interrupted or failed runs keep their files and save that status.

| Setting | Default |
| --- | --- |
| Input | One 12-bit ADC channel at 4000 Hz |
| Filter | Causal 4th-order high-pass 20 Hz, 4th-order low-pass 450 Hz, notch 50 Hz Q=30 |
| Settling | 250 ms after start/gap, then a complete window |
| Window / overlap | 200 / 50 ms, updated every 150 ms |
| Features | RMS, MAV, waveform length, zero crossings, slope-sign changes, sample variance |
| ZC/SSC threshold | 5 ADC counts; a starting value to evaluate |
| Sequence | Four windows, 650 ms history |
| LDA | Pooled covariance, fixed 0.1 diagonal shrinkage, equal class priors |
| ANN | Last feature window, 32 tanh units, four output logits |
| LSTM | One layer, 32 hidden units, four output logits from the last step |
| Neural training | 30 epochs, Adam 0.001, weight decay 0.0001, batch 64, inverse-frequency class weights, seed 42 |

Training and live prediction share the causal filter and feature code. No future
samples enter a window. Gaps reset filtering and sequence history. Windows with
more than 5% clipping are excluded. Scored windows must fit within one reviewed
phase; fast grip/release phases may have none. All models use the same eligible
endpoints. LDA/ANN use the last window; LSTM also uses preceding windows. History
length is not a measured response delay. Ordinary **Process EMG** remains a
separate offline, forward-backward analysis.

## Evaluate and select

Leave-one-subject-out evaluation keeps the test subject outside training.
Participants/trials are split before windowing. Scaling is fitted on training
data only. Epochs are fixed; test scores do not choose a stopping point or model.
Treat repeated comparisons as exploratory and reserve new subjects for final
evaluation after choosing settings.

Run `--calibration-trials 1` separately to include the first held-out-subject
trial by trial ID in training. Later trials are tested. Give IDs acquisition
order and keep at least one test trial. These adaptation scores must stay
separate from unseen-subject scores.

Results include four-class confusion matrices, support and macro F1, plus binary
open/closed scores. Absent classes have zero F1 and explicit zero support. False
closures/releases count changes between adjacent scored windows, not physical
robot failures. `first_matching_state_delay_s` runs from reviewed grip/release
onset to the first matching binary prediction within that phase. It does not
prove a new state transition occurred at that time. `unmatched_phases` counts
phases without a match. Gaps and phases with no eligible windows cannot
establish response delay or task success.
`test_label_coverage` lists every reviewed interval, including those with no
eligible window, so short phases are visible in the report.

The amplitude baseline replays the current estimator with saved pre-trial rest
and strong-grip calibration and a 0.5 binary threshold. Missing calibration is
reported as unavailable; test movements are never used to invent it. Compare
binary scores with binary scores: amplitude does not predict the four phases.
This baseline still requires personal pre-trial calibration.

In **Simulation → Live settings → Select reviewed EMG model**, choose a
candidate's `bundles/.../model.json`. Check participant scores and open the full
results. **Use this model for live EMG** records the review and selects it for
the next session. **Use calibrated amplitude** restores the default. New training
runs cannot select a model automatically.
**Plot participant scores** compares the saved phase and binary grip results.

Bundles freeze scaling, processing, feature order, labels and hashed weights.
A BLE sample-rate mismatch blocks model output; gaps/reconnects clear history.
Reconnects still require calibration. Predicted grip/hold allows the calibrated
amplitude; rest/release opens the hand. This estimates intent, not force or
individual finger positions. Live accuracy still needs subject testing.

## Basis

HT25's `src/emg/emg_processing_utils.py` supplies the six-feature starting point;
CDT406 EMG1 supplies the interval-review idea. The EMG3 report by Folkesson et al.
(2025) compares ANN/LSTM preprocessing and participant-held-out evaluation.
[Adewuyi et al. (2016)](https://doi.org/10.3389/fnbot.2016.00015) supports comparing
LDA and small networks. [Smith et al. (2011)](https://doi.org/10.1109/TNSRE.2010.2100828)
examines window/error/delay trade-offs. [Campbell et al. (2021)](https://doi.org/10.3389/fnins.2021.657958)
separates cross-user performance and personal adaptation. Their setups differ;
their accuracy figures do not transfer to AURORA.
