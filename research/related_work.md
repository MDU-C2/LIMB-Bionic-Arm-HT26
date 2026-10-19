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
[GRU](../src/ml/train_gru.py) learns camera movement profiles grouped by `user_id`.
An EMG classifier is not implemented yet.

The [guided trial](../src/recording/emg_trials.py) saves cue times and RMS/MAV
features. Cue times need checking against the actual movement. Offline export
uses forward-backward filtering; live control uses a causal filter. A deployed
classifier needs the same processing during training and live use.

Filter settings, calibration thresholds and fusion weights still need evaluation.
See the [sensor guide](../docs/SENSOR_DATA.md) for the current settings.

## Checks for future EMG models

- Keep raw recordings, reviewed labels, participant IDs and session details.
- Split by participant before making windows when testing generalisation to new
  people. Fit scaling and feature selection on training data only.
- Compare models using per-person scores, false closures and releases, response
  delay and task success. Check joint tracking against an independent angle reference.
- Save the model with its processing settings, calibration, data split and code version.

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
