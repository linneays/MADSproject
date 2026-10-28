# Ego-state complementary filter (MADS)

Course project for *Robotics Perception and Action*: a MADS filter plugin that
estimates the pose (x, y, δ) of the robotic walker by fusing wheel encoders,
gyroscope and HTC tracker with complementary filters.

## Files

| File | Content |
|---|---|
| `ego_state_filter.cpp` | The plugin. `process()` follows the 11 steps of the flowchart (slide 41) |
| `ego_state_filter.ini` | Example section for `mads.ini` (parameters and filter coefficients) |
| `my_filter.cpp` | Warm-up filter template from class (reference only) |

## Topics

- **Input:** `driver` (encoder counts), `calibrated_imu` (gyro), `htc` (absolute pose)
- **Output:** `ego_state_<group number>`, a 4×4 homogeneous matrix `H`

## Still to do

Search the code for `TODO`. The main ones:

- [ ] Filter cutoff frequencies and discrete coefficients (MATLAB `c2d`, Tustin) → `ego_state_filter.ini`
- [ ] Smooth functions `f_rho` and `f_xi` (maneuver-dependent weights, step 3)
- [ ] Gyroscope bias `b_G`
- [ ] Real wheel radius and wheel base
- [ ] Check HTC axes, encoder signs and the rotation convention of `H` against recorded data
- [ ] Test with a recording: `mads play --input FILE.bag --rate 1`
