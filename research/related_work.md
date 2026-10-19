# Related work and method

The main question is whether EMG can give reliable grip control across people
and arm positions, and whether IMU and camera input improve control. This note
links the proposed method to published work. The experiments below are planned;
they are not results from our system.

## What the studies support

| Part | Related work | What it means for this project |
| --- | --- | --- |
| Arm position | Fougner et al. [1] tested EMG classification in ten participants without limb loss. Training in multiple positions and adding accelerometer information reduced the position-related error in their setup. | Record the same grip task in several arm positions. Compare EMG alone with EMG plus IMU features. Our live IMUs currently estimate joint angles; they are not inputs to an EMG classifier. |
| Simple models | Adewuyi et al. [2] compared features and classifiers for partial-hand control in 16 participants without amputation and four with amputation. LDA performed comparably to the tested linear and nonlinear neural networks. | Include linear discriminant analysis (LDA) as a baseline. A more complex model needs a measured benefit. Their task used multiple EMG channels and different grasp classes. |
| Temporal models | Toro-Ossaba et al. [7] tested an LSTM with dense layers using four EMG channels for five gestures. | A compact LSTM is a candidate for comparison. Their channel count and gesture task differ from our single-channel grip control. |
| Other model candidates | Pizzolato et al. [3] compared acquisition setups using support vector machines (SVM) and Random Forests, with features including RMS and time-domain features. They separated movement repetitions for training and evaluation. | SVM and Random Forest are supported candidates for comparison. Their results do not select a winner for our single-channel signal or establish performance on unseen people. |
| Window length and delay | Smith et al. [4] tested a virtual prosthesis with 13 participants without limb loss. Longer windows reduced classification error but increased delay. Their best control performance occurred with 150-250 ms windows. | Our 200 ms feature window is a reasonable starting point to test. Measure control delay as well as classification scores. The current 50 ms overlap means a 150 ms step, not a 50 ms update interval. |
| Training across people | Campbell et al. [5] evaluated cross-user adaptation using leave-one-subject-out testing on intact-limb and amputee datasets. Their adapted neural model used a training repetition from the new user. | Compare a model with no target-user training against one with a separate short calibration recording. Keep those results separate. This supports testing adaptation; it does not establish that our GRU is suitable. |
| Camera and IMU fusion | Chen et al. [6] combined Kinect and IMU measurements using a Kalman filter for robotic arm control, with a data glove for wrist and finger input. | This is related work for combining visual and inertial measurements. Our OAK-D, MediaPipe and weighted angle fusion use different sensors and a different method. The paper does not validate our weights or angle accuracy. |

These studies guide the method. They do not establish rehabilitation outcomes
for stroke survivors or validate our hardware. We need to state the participant
population, channel count and task when comparing results.

## Current implementation

Live grip control uses filtered EMG amplitude and personal rest/strong-grip
calibration. There is no trained EMG grip model yet. The existing
[GRU training](../src/ml/train_gru.py) learns camera movement embeddings grouped
by `user_id`; it does not learn EMG grip intention.

The [guided trial](../src/recording/emg_trials.py) saves rest, grip, hold and
release cue times beside raw EMG and IMU recordings. It exports RMS and mean
absolute value (MAV). Camera recordings are saved separately with timing data.
The saved cues describe instructions, not verified movement onset. Review the
video and correct labels before treating the four phases as ground truth.
Grip and hold may have similar amplitude, so four separable classes are still
a hypothesis. Without a force reference, we can evaluate hand state and control
behaviour, not grip-force accuracy.

The 20-450 Hz band-pass, 50 Hz notch, 80 ms live envelope, calibration thresholds,
25% camera weight and trial timings are current engineering settings. They
need explicit justification or experiments; the papers above do not validate
those exact settings. Offline feature export uses forward-backward filtering;
live control uses a causal filter. Any deployed classifier must be trained and
evaluated with the same causal processing it will receive live.

## Proposed evaluation

1. Check repeated rest and contraction recordings before collecting the dataset.
   The [signal check](../docs/EMG_SIGNAL_CHECK.md) found only about an 8% RMS
   increase in one requested-grip capture. That is a local observation, not a
   reliable accuracy estimate. Check contact, gain, clipping and packet loss.
2. Use participant IDs and repeated sessions. Record arm position,
   sensor placement, electrode reapplication and calibration details. Keep raw
   recordings, reviewed labels and processing settings so features can be rebuilt.
   The current recorder does not collect all of this placement metadata yet.
3. Start with rest versus active contraction. Compare calibrated amplitude
   control against LDA, a small ANN, SVM and Random Forest using the same recordings. Extend
   RMS/MAV with a justified time-domain feature set if needed. Evaluate grip,
   hold and release classification only after checking the transition labels.
   Test a compact LSTM from the earlier work if simpler methods miss useful dynamics.
4. Split by participant before creating windows. Hold one person out for each
   test fold and tune settings using only the remaining people. Keep entire
   sessions and repetitions together. Fit scaling and feature selection on
   training data only. For personalised control, use a separate calibration
   recording from the test person and evaluate on later recordings.
5. Report per-person macro-F1, balanced accuracy, a confusion matrix, false
   closures and releases, and response delay. Then measure cup-task success and
   completion time in the simulator. Compare EMG alone with EMG plus IMU inputs.
   Compare camera-only, IMU-only and fused joint tracking against an independent
   angle reference, including camera occlusion and sensor drift.
6. Save the chosen model with the code version, participant/session split,
   feature definitions, filtering and calibration settings, and evaluation
   results. Record failures as well as successful trials.

This evaluation pipeline is not implemented yet. Choose the model after these
comparisons and keep results from our experiments separate from published findings.

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
