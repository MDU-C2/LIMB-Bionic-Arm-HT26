# Related work

These studies cover EMG processing, movement recognition and sensor fusion.
The LSTM study by Toro-Ossaba et al. [7] was cited in the earlier EMG3 work by Folkesson et al.
(2025), *sEMG-Based Gesture Classification for Real-Time Embedded Prosthetic
Control*. References [1-6] are additional literature.

## Studies

| Topic | Finding | Use in this project |
| --- | --- | --- |
| Arm position | Fougner et al. [1] reduced position-related classification error through training in several positions and using accelerometer information. | Compare EMG alone with EMG and IMU inputs across arm positions. |
| Simple classifiers | Adewuyi et al. [2] found that LDA performed comparably to their tested neural networks for partial-hand grasp classification. | Include LDA as a baseline. |
| Model comparison | Pizzolato et al. [3] compared SVM and Random Forest classifiers using several EMG features and acquisition setups. | Compare candidates on the same recordings and splits. |
| Window length | Smith et al. [4] found a trade-off between classification error and delay. Their best control performance used 150-250 ms windows. | Test the current 200 ms window alongside its response delay. |
| Different participants | Campbell et al. [5] evaluated cross-user adaptation with a separate training repetition from the held-out participant. | Evaluate models with and without personal calibration separately. |
| Camera and IMU | Chen et al. [6] fused Kinect and IMU measurements with a Kalman filter for robotic arm control. | Compare visual, inertial and combined tracking. Their method differs from our weighted angle fusion. |
| Temporal models | Toro-Ossaba et al. [7] used a compact LSTM with four EMG channels to recognise five gestures. | Consider an LSTM for comparison, accounting for our single-channel setup. |

Compare results with their participant groups, channel counts and tasks in mind.
These studies do not establish accuracy or rehabilitation outcomes for our setup.

## Current implementation

Live grip control uses EMG amplitude with rest and strong-grip calibration.
IMU and camera estimates supply joint angles. The existing
[GRU](../src/aurora/ml/train_gru.py) learns camera movement profiles grouped by `subject_id` (older files use `user_id`).
The [EMG comparison](../docs/emg-learning.md) trains LDA, ANN and LSTM from
reviewed raw recordings. It saves participant splits, scaling and settings.
Models need explicit review and selection before live use.

The [guided trial](../src/aurora/recording/emg_trials.py) saves cue times and six time-domain
features. Cue times need checking against the actual movement. Offline export
uses forward-backward filtering; live control uses a causal filter. A deployed
classifier therefore uses one shared causal pipeline for training and live use.

The [fusion comparison](../docs/fusion-comparison.md) tests weighted,
complementary, camera Kalman and planar EKF baselines offline. It requires
synthetic motion or a described independent reference. Live fusion keeps its
current correction until measured comparisons support a change.

Filter settings, calibration thresholds and fusion weights still need evaluation.
See the [sensor guide](../docs/sensor-data.md) for the current settings.

## Evaluation checks

- Keep raw recordings, reviewed labels, participant IDs and session details.
- Split by participant before making windows when testing generalisation to new
  people. Fit scaling and feature selection on training data only.
- Compare models using per-person scores, false closures and releases, response
  delay and task success. Check joint tracking against an independent angle reference.
- Save the model with its processing settings, calibration, data split and code version.

## Recording protocols

The [cup protocol](../docs/cup-transfer-protocol.md) keeps Oscar Ågren's (2026)
four-marker task, three-second countdown and eight-second recording window,
with separate natural, slow and fast trials. Oscar used four pace conditions;
the three conditions here are a project choice. His thesis describes vision
capture with calibrated stereo, not wearable IMU or EMG calibration.
The earlier HT25 EMG recorder explicitly separated rest/action/rest. Its report
describes EMG training and separate human-IMU direction experiments, not a
validated synchronized multimodal training dataset. Jonas's
AURORA recorder saves raw EMG and IMU samples with their acquisition timestamps.
The BLE recorder keeps this raw-data approach with separate files for each
sampling rate. Cup-trial controls are for the operator; the subject follows
printed instructions and receives the start signal from the operator.

- [HANDdata (2023)](https://doi.org/10.1038/s41597-023-02313-w) used practice
  runs, audio start/stop cues and a repeat control. These support the guided
  preparation and retry flow. AURORA keeps rejected attempts rather than deleting them.
  HANDdata also saved calibration reference data for each participant before the
  tasks. AURORA saves a separate subject reference with raw signals and video.
- [McInnes et al. (2025)](https://doi.org/10.1109/JTEHM.2025.3565986) estimated
  functional joint axes for sensor-to-segment calibration, using custom mounts,
  inertial orientation data and optical validation. This distinguishes full
  sensor-to-arm alignment from simple offset zeroing. Their method and accuracy
  are not implemented or established for AURORA's two six-axis sensors. The
  preparation here uses a different constrained functional fit, described below.
- [Reach&Grasp (2025)](https://doi.org/10.1038/s41597-025-04552-5) allowed
  unconstrained reach-and-grasp timing and segmented repetitions afterwards.
  This supports reviewing actual movement timing after capture. The capture
  window here is adjustable so slow movements can finish without changing pace.
- [An EEG-EMG-kinematics dataset from wrist pointing tasks (2026)](https://doi.org/10.1038/s41597-026-07287-z)
  recorded continuous signals and timestamps for preparation and movement
  sections. AURORA likewise saves cues separately from raw samples and
  movement labels; cues do not automatically label actual movement.

The tasks, sensors and participants differ. The default three-second countdown
and eight-second capture follow Oscar's workflow. The 20-second rest, preparation
durations and initial three trials per pace are project collection settings,
not a validated universal protocol. Pilot the window before collecting subjects,
extend it if needed and keep the selected settings with each recording.
The reference step durations are also project settings. Reference completion is
separate from anatomical alignment and signal-quality validation. Jonas's raw
recorder and the earlier HT25 offset calibration do not provide full
sensor-to-segment calibration for this mounting.

Earlier project sources: Oscar Agren (2026), *Personalized, Vision-Based
Human-Like Robotic Arm Control on Real-Time Embedded Systems*, sections 5.3,
7.2 and 7.4.1; and the HT25 team report *System Design of an Intent-Driven
Robotic Arm with Adaptive Grasping Using EMG, Vision, and Pressure Sensor*
(22 February 2026), sections 5.3, 8.4 and 9.4. The earlier
[HT25 repository](https://github.com/MDU-C2/LIMB-HT25) contains the EMG recorder
and IMU experiments. The report's date and repository year differ.

## Implemented subject alignment

The recording workflow uses isolated arm raises, elbow bends and forearm turns.
The functional-axis approach follows de Vries et al. (2010), *Functionally
interpretable local coordinate systems for the upper extremity using inertial
and magnetic measurement systems*, [doi:10.1016/j.jbiomech.2010.03.007](https://doi.org/10.1016/j.jbiomech.2010.03.007).
McInnes et al. discuss this constrained-movement approach and the need for a
second direction to fully define a sensor-to-segment rotation. Their own method
uses a different optimisation, magnetic orientation data, three sensors and
custom mounts; it is not reproduced here.

Our implementation estimates dominant gyro axes with an eigenvector fit during
isolated motions. The arm raise supplies the upper-arm flexion axis. The still
reference supplies its longitudinal direction through gravity, under the
explicit assumption that the upper arm rests beside an upright body. Forearm
bends and turns supply its flexion and longitudinal axes. The resulting bases
are orthonormal proper rotations, with X positive flexion, Y distal and Z = X
cross Y. The turn-axis sign is resolved against gravity and the thumb-up pose.
Stereo shoulder/elbow/wrist points check reference geometry, first movement
direction and the return pose; hidden hips do not supply inferred landmarks.

This is a project adaptation. Stillness, timing, movement-range, axis-consistency
and camera checks are collection checks to evaluate in a physical pilot, not
proof of anatomical accuracy. Their settings and diagnostics are saved in
`alignment.json`, with the source file hashes. Aligned IMU exports are separate
from raw data. The calibration does not independently measure trunk motion,
remove all soft-tissue effects or eliminate six-axis heading drift. Physical
angle validation is still required.

## References

1. Fougner, A., Scheme, E., Chan, A. D. C., Englehart, K., and Stavdahl, O.
   (2011). *Resolving the limb position effect in myoelectric pattern recognition*.
   IEEE Transactions on Neural Systems and Rehabilitation Engineering, 19(6),
   644-651. [doi:10.1109/TNSRE.2011.2163529](https://doi.org/10.1109/TNSRE.2011.2163529).
2. Adewuyi, A. A., Hargrove, L. J., and Kuiken, T. A. (2016).
   *Evaluating EMG Feature and Classifier Selection for Application to
   Partial-Hand Prosthesis Control*. Frontiers in Neurorobotics, 10, 15.
   [doi:10.3389/fnbot.2016.00015](https://doi.org/10.3389/fnbot.2016.00015).
3. Pizzolato, S., Tagliapietra, L., Cognolato, M., Reggiani, M., Müller, H.,
   and Atzori, M. (2017). *Comparison of six electromyography acquisition setups
   on hand movement classification tasks*. PLOS ONE, 12(10), e0186132.
   [doi:10.1371/journal.pone.0186132](https://doi.org/10.1371/journal.pone.0186132).
4. Smith, L. H., Hargrove, L. J., Lock, B. A., and Kuiken, T. A. (2011).
   *Determining the optimal window length for pattern recognition-based
   myoelectric control: balancing the competing effects of classification error
   and controller delay*. IEEE Transactions on Neural Systems and Rehabilitation
   Engineering, 19(2), 186-192.
   [doi:10.1109/TNSRE.2010.2100828](https://doi.org/10.1109/TNSRE.2010.2100828).
5. Campbell, E., Phinyomark, A., and Scheme, E. (2021).
   *Deep Cross-User Models Reduce the Training Burden in Myoelectric Control*.
   Frontiers in Neuroscience, 15, 657958.
   [doi:10.3389/fnins.2021.657958](https://doi.org/10.3389/fnins.2021.657958).
6. Chen, F., Wang, F., Dong, Y., Yong, Q., Yang, X., Zheng, L., Gao, Y., and
   Su, H. (2023). *Sensor Fusion-Based Anthropomorphic Control of a Robotic Arm*.
   Bioengineering, 10(11), 1243.
   [doi:10.3390/bioengineering10111243](https://doi.org/10.3390/bioengineering10111243).
7. Toro-Ossaba, A., Jaramillo-Tigreros, J., Tejada, J. C., Peña, A.,
   López-González, A., and Castanho, R. A. (2022). *LSTM Recurrent Neural Network
   for Hand Gesture Recognition Using EMG Signals*. Applied Sciences, 12(19), 9700.
   [doi:10.3390/app12199700](https://doi.org/10.3390/app12199700).
