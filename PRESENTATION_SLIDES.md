# Kalman Filter Project — Presentation Content (25 min, 15–20 slides)

**Audience:** SpaceX SWE  
**Use:** Copy bullets and visuals into PowerPoint/Google Slides; adjust to your style.

---

## 1. Introduction / Background (1–2 slides)

### Slide 1: About Me
- **Short professional intro:** Year, major/role, relevant experience
- **Highlight:** Illinois Space Society GNC team; rocket state estimation
- **Previous work:** EKF design, simulation (SIL), sensor fusion
- **Visual:** Photo of you or team / rocket project

### Slide 2: Motivation
- **Why this project?** Rocket state estimation is safety-critical: apogee detection, recovery, staging all depend on accurate position/velocity.
- **Challenge:** Noisy sensors (IMU, barometer, GPS), high dynamics, real-time constraints.
- **One-liner:** “Fuse barometer + GPS + IMU into a single, consistent state estimate for flight logic.”
- **Visual:** Simple diagram: Rocket with sensors (IMU, Baro, GPS) → KF → State estimate → Control/FSM

---

## 2. Project Scope & Problem Statement (2–3 slides)

### Slide 3: Problem Statement
- **Objective:** Estimate 3D position and velocity in real time during flight.
- **State:** Position (x = altitude, y = east, z = north) and velocity (vx, vy, vz). Acceleration is a **control input**, not a state.
- **Constraints:** Noisy sensors, GPS dropouts at high speed/altitude, barometer sensitivity to weather and venting.
- **Visual:** Block diagram: [Sensors] → [EKF] → [State estimate] → [FSM / recovery / staging]

### Slide 4: Requirements & Challenges
| Requirement | Target | Challenge |
|-------------|--------|-----------|
| Update rate | Real-time (e.g. 20–50 Hz) | Must run on flight hardware |
| Position accuracy | Meters (horizontal), ~meters (altitude) | GPS loss during boost; baro bias |
| Velocity accuracy | m/s | No direct velocity sensor; from integration + GPS |
| Robustness | Sensor dropouts | GPS fix_type; baro glitches; LANDED jitter |
- **Safety-critical:** Apogee, staging, and recovery depend on this estimate.

### Slide 5: Team & Timeline
- **Team:** GNC EKF team (names/roles if you want).
- **Phases:** Design (state/model choice) → Implementation (C++ EKF) → SIL (CSV replay) → Flight integration.
- **Visual:** Simple Gantt or timeline: Design → Sim → Flight → Analysis.

---

## 3. Solution / Implementation (5–8 slides)

### Slide 6: Kalman Filter Overview
- **Model:** **Linear** KF (dynamics are linear; we use “EKF” in name but state propagation is F·x + B·u).
- **State vector (6):** `x = [x, vx, y, vy, z, vz]ᵀ` — position and velocity only; acceleration is **control input u = [ax, ay, az]ᵀ**.
- **Flow:** Predict with F, B, u → Barometer update → GPS update (when valid).
- **Visual:** [IMU + GPS + Barometer] → [Predict: F,B,u] → [Update: Baro, GPS] → [State + Covariance P]

### Slide 7: State Vector & Matrices
- **State:** `[x, vx, y, vy, z, vz]` — x = altitude (up), y = east, z = north.
- **F (6×6):** Position integrates velocity; velocity unchanged in F (acceleration via B·u).
  - `x += vx*dt`, `y += vy*dt`, `z += vz*dt`; vx, vy, vz updated by B·u.
- **B (6×3):** Maps `u = [ax, ay, az]` to velocity derivatives: `vx += ax*dt`, `vy += ay*dt`, `vz += az*dt`.
- **H (4×6):** Barometer measures x (row 0); GPS measures x, y, z (rows 1–3).
- **Visual:** Small matrix sketch: F with dt in (0,1), (2,3), (4,5); B with dt in (1,0), (3,1), (5,2).

### Slide 8: Process & Measurement Noise
- **Q (process noise):** Models uncertainty in dynamics (unmodeled acceleration, drag, IMU errors). We use **per-axis 2×2 blocks** (position–velocity) driven by acceleration uncertainty σ_a ≈ 0.2 m/s²:
  - Position variance ∝ dt⁴/4 · σ_a², velocity ∝ dt² · σ_a², with cross terms. Tuned so P doesn’t collapse or explode.
- **R (measurement noise):** Diagonal. Barometer ~1.5 m (R ≈ 2.25 m²), GPS altitude ~3 m (R ≈ 9 m²), GPS horizontal ~2 m (R ≈ 4 m²). Reflects sensor quality.
- **Why it matters:** Q too small → overconfident (P too small); R wrong → filter trusts sensors wrong. We tuned R from typical baro/GPS specs.
- **Visual:** Optional: trajectory with/without proper Q, or final P_k (e.g. from your sim) showing realistic uncertainties.

### Slide 9: Control Input Handling
- **u = acceleration** from IMU: body-frame accelerometer → rotate to global (BodyToGlobal) → convert g to m/s² (gravity only when FSM > IDLE). Bias corrections (e.g. +0.045, -0.065, -0.06) applied in body frame.
- **Single place:** u computed once in **priori()**, stored in `u_control_last_`; used for both prediction and for output “estimated acceleration.”
- **Prediction:** `x_priori = F * x_k + B * u`; `P_priori = F * P_k * F' + Q`.
- **Code snippet (conceptual):**
  ```text
  BodyToGlobal(orientation, accel_body);
  u = accel_global * g_ms2;   // gravity on after IDLE
  x_priori = F * x_k + B * u;
  P_priori = F * P_k * F' + Q;
  ```

### Slide 9b: BodyToGlobal — How It Works (Detail)
- **Purpose:** Convert a 3D vector from the **body frame** (rocket axes: e.g. forward, right, down) to the **global frame** (world: up = X, east = Y, north = Z) using the rocket’s orientation (roll, pitch, yaw).
- **Why:** The state [x, vx, y, vy, z, vz] is in global frame. Acceleration must be in the same frame so that “velocity += acceleration × dt” is correct. The IMU measures in body frame, so we rotate that vector into global.
- **Steps:**
  1. **Three rotation matrices** (Euler angles, radians):
     - **Roll:** rotation about the body axis that ends up as “roll” in your convention (in code: about Z in the intermediate frame).
     - **Pitch:** rotation about the body lateral axis (about Y).
     - **Yaw:** rotation about the body longitudinal/vertical axis (about X in code).
  2. **Combine:** `R = yaw * pitch * roll` (matrix multiply left-to-right = apply roll first, then pitch, then yaw in body frame).
  3. **Apply:** `temp = R * body_vec` → **temp** is the same physical vector as `body_vec`, but expressed in the rotation’s output frame (axes may still be Z-up depending on convention).
  4. **Convention fix (if needed):** Your global frame is X = up, Y = east, Z = north. If the rotation output is Z-up, we copy/permute/negate components so the result is stored in `body_vec` as (global_x, global_y, global_z).
- **Math:** Each of roll, pitch, yaw is a 3×3 rotation matrix (cos/sin of one angle). `R * v` gives the coordinates of the vector `v` in the frame that the rotation maps *to*.
- **Visual:** Diagram: body axes (forward, right, down) + roll/pitch/yaw → global axes (up, east, north); one vector drawn in both frames.

### Slide 10: Sensor Fusion & Updates
- **Barometer:** One measurement (altitude = x). Innovation = baro_alt - x_priori(0). Kalman gain K_baro (6×1), then `x_k = x_priori + K_baro * innovation`, `P_k = (I - K_baro*H_baro)*P_priori`.
- **GPS:** Only when `fix_type` valid and lat/lon non-zero. Lat/lon/alt → ECEF → ENU relative to **launch origin** (set in IDLE via `reference_GPS`). GPS gives 3 measurements: x (altitude), y (east), z (north). Same update form with 3×3 innovation covariance S_gps and 6×3 K_gps. Skip update if no fix.
- **Order in code:** Barometer update first, then `compute_gps_inputs()` (GPS update) inside `update()`.
- **Visual:** Flow: Baro → update x,P → GPS (if valid) → update x,P again.

### Slide 11: Simulation & Testing
- **SIL:** Replay flight CSV (timestamps, baro, IMU, GPS, FSM) through EKF; no live sensors. `test_ekf.cpp` loads CSV, runs `ekf.tick(dt, ...)` per row, records state to CSV.
- **Ground truth:** From same flight data (e.g. raw baro, GPS-derived position). Compare EKF position/velocity vs raw or post-processed truth.
- **Plots:** Position (x,y,z) and velocity over time; error vs time; optional covariance (e.g. √P(0,0), √P(2,2)) over time.
- **Corner cases:** GPS dropout (no update when fix lost); high acceleration (B·u dominant); IDLE on pad (reference_GPS, baro init); LANDED (velocity reset after debounce to avoid jitter).

### Slide 12: Real Flight Results (if applicable)
- **EKF in loop:** Same C++ EKF runs on flight hardware; receives baro, IMU, GPS at runtime.
- **Plots:** x, y, z and vx, vy, vz from EKF over flight timeline; overlay FSM (boost, coast, apogee, recovery).
- **Visual:** Launch photo or short clip + trajectory plot.

### Slide 13: Block Diagram Recap
- **One slide:** Sensors (Baro, IMU, GPS) → Orientation (for BodyToGlobal) → **Priori** (F, B, u → x_priori, P_priori) → **Update** (Baro then GPS → x_k, P_k) → State output (position, velocity, accel from u) → Downstream (FSM, recovery, logging).
- **Visual:** Single block diagram with all inputs/outputs.

---

## 4. Results & Outcomes (2–3 slides)

### Slide 14: Accuracy Metrics
- **From SIL:** e.g. RMSE position (m), RMSE velocity (m/s) vs ground truth over a flight replay. Mention which “truth” (baro altitude, GPS position).
- **From flight (if any):** Same metrics if you have post-flight reference (e.g. GPS-only or smoothed trajectory).
- **Visual:** Table or bar chart: RMSE x, y, z; RMSE vx, vy, vz.

### Slide 15: Real-Time Performance
- **Rate:** e.g. 20–50 Hz (match your dt in SIL and flight).
- **Implementation:** C++, Eigen for matrices; fixed 6-state, minimal allocations in the hot path (no dynamic alloc in priori/update).
- **Visual:** Optional: CPU usage or execution-time histogram per tick.

### Slide 16: Lessons Learned
- **Challenges:** Tuning Q/R without ground truth; GPS dropout and when to skip update; baro vs GPS altitude consistency; LANDED state causing velocity jitter.
- **Fixes:** reference_GPS at IDLE for stable ENU origin; GPS update only when fix valid; debounced LANDED velocity reset (e.g. 0.5 s); acceleration as control input (B·u) instead of state to avoid velocity divergence.
- **Robustness:** Single code path for SIL and flight; same R values tuned to sensor specs (baro ~1.5 m, GPS horiz ~2 m, GPS alt ~3 m).

---

## 5. Summary / Takeaways (1–2 slides)

### Slide 17: Key Contributions
- **Your role:** EKF design (6-state, F/B/Q/R/H), control-input handling, GPS/baro fusion, SIL pipeline, tuning.
- **Design choices:** Linear propagation; acceleration as u; Singer-like Q per axis; combined H/R for baro + GPS; reference_GPS for ENU.
- **Deliverables:** C++ EKF in GNC repo, SIL with CSV replay and plotting, documentation (e.g. EKF_EXPLANATION.md).

### Slide 18: Impact
- **Impact:** Reliable state estimate → better apogee detection, staging, and recovery decisions; foundation for future GNC (e.g. nav for guidance).
- **Takeaway:** Rigorous sensor fusion and tuning (Q/R, reference frame, dropout handling) are essential for safety-critical rocket software.
- **Visual:** Optional: team/project photo or “Thank you / Questions.”

---

## Troubleshooting: Sim Performs Better *Without* BodyToGlobal

If your SIL state estimate looks better when you **don’t** rotate acceleration (i.e. skip BodyToGlobal and use body-frame accel as if it were global), one of the following is likely wrong. Fix the cause; don’t leave BodyToGlobal off for flight.

### 1. Sim / CSV acceleration is already in global frame
- **Symptom:** With BodyToGlobal, velocity or position drifts or looks wrong; without it, it tracks well.
- **Cause:** The SIL CSV (or the pipeline that produced it) may already give acceleration in **world frame** (e.g. post-processed or from a sim that outputs global accel). Rotating it again with BodyToGlobal is then a **double rotation** and corrupts the signal.
- **Fix:** Check how the CSV accel is defined (body vs global). If it’s global, **skip** BodyToGlobal in the code path that reads that CSV (e.g. flag or separate SIL branch), but **keep** BodyToGlobal for real flight where the IMU is body-frame.

### 2. Orientation convention or axes don’t match
- **Symptom:** With BodyToGlobal, the estimate is worse; without it, it’s better.
- **Cause:** Your `rotation.h` uses a specific roll/pitch/yaw **axis assignment** and **order** (e.g. roll about Z, pitch about Y, yaw about X, with `R = yaw * pitch * roll`). The sim or CSV may use a different convention (e.g. aerospace: roll about X, pitch about Y, yaw about Z). If the names match but the axes don’t, the rotation is wrong.
- **Fix:** Align the rotation with the data source. Compare with `gnc/old_filters/yessir.cpp`, which uses roll about X, pitch about Y, yaw about Z. If your sim uses that convention, use the same axes and order in `rotation.h`, or convert Euler angles from sim convention to your convention before calling BodyToGlobal.

### 3. Orientation units: radians vs degrees
- **Symptom:** With BodyToGlobal, behavior is erratic or way off.
- **Cause:** `BodyToGlobal` uses `cos(angle)`, `sin(angle)` and assumes **radians**. If the CSV or `getEuler()` returns **degrees**, then e.g. 90° is interpreted as 90 rad and the rotation is wrong.
- **Fix:** Ensure orientation is in **radians** before calling BodyToGlobal. If the CSV or Orientation stores degrees, convert: `angle_rad = angle_deg * (π/180)` before building the rotation matrices.

### 4. Optional: Bypass for SIL only
- To compare “with vs without” cleanly, add a flag (e.g. compile-time or runtime) that skips BodyToGlobal only in the SIL path, so you use body-frame accel as u in sim but keep BodyToGlobal for flight. Document that the flag is for debugging only and must be off for real hardware.

---

## Quick Reference — Your Actual Numbers (for Q&A)

- **State:** 6: `[x, vx, y, vy, z, vz]`.
- **F:** Identity + dt at (0,1), (2,3), (4,5). **B:** dt at (1,0), (3,1), (5,2).
- **Q:** σ_a = 0.2 m/s²; per-axis blocks with dt⁴/4, dt³/2, dt².
- **R (current):** Baro 2.25, GPS alt 9, GPS east/north 4 (m²).
- **H:** Row 0: x (baro); rows 1–3: x, y, z (GPS).
- **Reference:** `reference_GPS` in IDLE sets launch origin for ENU; GPS update skipped if no fix.

Use this doc as the source for slide text and talking points; adjust numbers and emphasis to match what you actually present (e.g. if you have real flight RMSE, plug it in on Slide 14).
