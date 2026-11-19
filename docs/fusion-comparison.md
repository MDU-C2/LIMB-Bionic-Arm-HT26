# Offline fusion comparisons

```powershell
python -m aurora.sensors.fusion_comparison --synthetic
```

This saves `comparison.json`, estimates and a plot under `outputs/diagnostics/`.
The known trajectory includes a camera dropout. It checks algorithm behaviour;
it does not measure accuracy on a person or change live fusion.

The comparison uses one calibrated planar sensor axis:

- Gyro integration with weighted camera correction, following the current
  correction idea at 0.25 per 30 Hz observation.
- Complementary gyro/gravity correction, starting at HT25's 0.98 weight per
  100 Hz sample, plus camera correction.
- Constant-velocity camera Kalman smoothing, following LIMB1's idea with actual
  time steps and angle units.
- A planar angle/gyro-bias EKF with nonlinear `[sin(angle), cos(angle)]` gravity
  observations and camera angle correction.

These are isolated baselines. They do not reproduce the whole live controller
or HT25's 13-state AprilTag position/quaternion EKF. That older pipeline needs
frames and metric measurements unavailable in ordinary RGB recordings. This
comparison does not estimate hand position, two-sensor elbow angles or heading.

For measured data, provide an aligned CSV:

```text
timestamp_s,gyro_dps,gravity_sin_g,gravity_cos_g,camera_deg,reference_deg
```

Camera values may be blank between fresh observations. All sources must share
the calibrated planar axis, sign and clock. Gravity components use g. Motion
acceleration can invalidate the gravity model even with a norm near 1 g.
Timestamps must increase, with no gap above 100 ms; samples are not interpolated.

Beside `angles.csv`, save `angles.json`:

```json
{
  "independent_reference": true,
  "reference_source": "describe the separate angle reference and alignment",
  "clock_domain": "describe the common clock",
  "frame": "calibrated_planar_sensor_axis"
}
```

Run `python -m aurora.sensors.fusion_comparison --file PATH/angles.csv`.
The operator must verify the reference and alignment. Reusing the camera input
as its own reference does not establish tracking accuracy.

The report saves settings, RMSE, MAE and 95th-percentile absolute error. Best
temporal shift over ±250 ms is an offline alignment diagnostic, not compute
latency. Stationary references have no shift estimate. Human angles stay unclipped.

Earlier HT25 complementary/EKF and LIMB1 Kalman code provide the comparison
ideas. [Chen et al. (2023)](https://doi.org/10.3390/bioengineering10111243) studies
camera/IMU Kalman fusion for arm control with a different measurement setup.
No previous accuracy figure is assumed to apply here.
