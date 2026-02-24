# Rangefinder — Theory of Operation

## 1. System Architecture

The Rangefinder implements a **semantic source selection** architecture organized into eight functional layers. Instead of weighted-average fusion, a priority-based state machine selects ONE authoritative depth source per frame, with dual Kalman filters tracking foreground and background hypotheses independently.

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 8: UI Presentation                                    │
│  RangefinderView → HUDOverlayView → BackgroundRangeChip    │
│  FFPReticleView(3 styles) → StadiametricBracketOverlay     │
│  OperatorGuidanceView → MapPiPView → SettingsView          │
├─────────────────────────────────────────────────────────────┤
│  Layer 7: Operator Guidance                                  │
│  OperatorGuidanceEngine (IMU stability + coaching hints)     │
├─────────────────────────────────────────────────────────────┤
│  Layer 6: Output Processing                                  │
│  BallisticsSolver → InclinationCorrector                     │
├─────────────────────────────────────────────────────────────┤
│  Layer 5: Temporal Filtering (Dual Kalman)                   │
│  fgKalmanFilter + bgKalmanFilter → MotionAwareSmoother      │
│  Source-switch reset → Outlier Reject (DEM bypass)           │
├─────────────────────────────────────────────────────────────┤
│  Layer 4: Semantic Source Selection (State Machine)          │
│  semanticSelect(): Stadia > LiDAR > Object > DEM > Neural  │
│  Primary + Background hypothesis → SemanticSourceDecision    │
├─────────────────────────────────────────────────────────────┤
│  Layer 3: Depth Source Estimation                            │
│  LiDAR │ Neural+Cal │ Geometric │ DEM │ Object │ Stadia    │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: Sensor Abstraction                                 │
│  CameraManager │ InclinationManager │ LocationManager        │
│  ARSession     │ CMMotionManager    │ CLLocation+CMAltimeter │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: Hardware                                           │
│  LiDAR │ Camera │ IMU │ GPS │ Magnetometer │ Barometer      │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow (Per Frame)

```
ARFrame (60 Hz)
  │
  ├─→ LiDAR depth map ──→ LiDARDepthProvider ──→ ┐
  ├─→ Camera image ─────→ MetricDepthEstimator ──→ │
  │                        (15 Hz, CoreML)          │
  ├─→ Calibration pairs → ContinuousCalibrator ──→ │
  │                                                  ├─→ UnifiedDepthField
  ├─→ IMU pitch ────────→ GeometricEstimator ─────→ │    semanticSelect()
  │                                                  │         │
  ├─→ GPS + heading ────→ DEMRaycastEstimator ────→ │    ┌────┴────┐
  │                        (2 Hz, async)             │    ↓         ↓
  ├─→ Camera image ─────→ ObjectDetector ─────────→ │  Primary  Background
  │                        (5 Hz, CoreML+Vision)     │    │         │
  └─→ User bracket ─────→ StadiametricInput ───────→ ┘    ↓         ↓
                                                     fgKalman   bgKalman
                                                        │         │
  IMU angular velocity ──→ OperatorGuidanceEngine       ↓         ↓
   (30 Hz, gyroscope)      ├─→ StabilityLevel     Inclination    ↓
                            ├─→ GuidanceHints      Corrector  bgRange
                            ├─→ ReadingLock            │
                            └─→ RespiratoryPause       ↓
                                                  RangeOutput ──→ feedRange()
                                                       │            to Guidance
                                                       ↓
                                              OperatorGuidanceView
                                              (StabilityBar + HintChips)
```

---

## 2. Depth Sources

### 2.1 LiDAR (0.3–10m)

The iPhone LiDAR scanner provides direct time-of-flight depth measurements via ARKit. The depth map and confidence map are extracted from each ARFrame.

**Characteristics:**
- Reliable range: 0.3–5m (sweet spot), usable to 8–10m
- Returns a dense depth map at camera resolution
- Confidence map distinguishes high/medium/low quality pixels
- Unaffected by lighting conditions
- Sampled at the crosshair point (screen center 0.5, 0.5)

**Confidence curve:**
- 0.3–3m: 0.98 (peak — direct sensor measurement)
- 3–5m: 0.98 → 0.70 (linear falloff)
- 5–8m: 0.70 → 0.20 (rapid degradation)
- 8–10m: 0.20 → 0.0 (tail, unreliable)
- Beyond 10m: 0.0

**Role in system:** Ground truth for neural model calibration. Primary ranging source within 5m. Feeds continuous calibration pairs to `ContinuousCalibrator`.

### 2.2 Neural Depth Estimation (2–150m)

DepthAnythingV2 (Small, Float16) is a monocular depth estimation model that produces relative/inverse depth maps from single camera images. The raw output is not metric — it requires calibration.

**Pipeline:**
1. Camera image resized and normalized for CoreML
2. Model inference at 15 FPS (67ms budget)
3. Output: inverse depth map (high values = close, low values = far)
4. Calibration: `ContinuousCalibrator` converts inverse depth to metric meters
5. Calibrated value sampled at crosshair

**Calibration (ContinuousCalibrator):**
The calibrator maintains a rolling window of 50 (neural_depth, lidar_depth) pairs collected in the 0.2–8m LiDAR overlap zone. It fits a weighted least-squares model:

- Inverse model: `lidar = scale / neural + shift`
- Exponential recency weighting: older samples decay by factor 0.95 per second
- Confidence = R-squared of fit x sample completeness
- Model type auto-detected via Pearson correlation (forced to inverse for DepthAnythingV2)

**Confidence curve (distance-dependent, with 50m hard cap):**
- 2–5m: 0.3 → 0.8 (ramping, LiDAR is better here)
- 5–8m: 0.8 → 0.9 (near calibration data)
- 8–15m: 0.9 (close to training range)
- 15–25m: 0.9 → 0.70 (moderate extrapolation)
- 25–40m: 0.70 → 0.45 (significant extrapolation)
- 40–50m: 0.45 → 0.35 (approaching hard cap)
- **≥ 50m: 0.0 (hard cap — `AppConfiguration.neuralHardCapMeters`)**

The hard cap at 50m replaces the gradual fade to a 0.10 floor that existed in the original architecture. This prevents neural from silently contributing wrong estimates at ranges where inverse-depth noise amplification makes the readings unreliable.

**Calibration quality modifier:** Applied on top of the distance curve. Fresh calibration (<30s) gets full quality. Quality decays slowly: 30–90s loses up to 15%, 90–300s loses up to 40%. Floor at 0.30 (never fully distrusted).

### 2.3 Geometric Ground-Plane Ranging (5–800m)

Uses the IMU pitch angle and known camera height to compute ground-plane distance via trigonometry.

**Formula:** `D = cameraHeight / tan(pitchBelowHorizontal)`

**Assumptions:**
- Target is on or near a flat ground plane
- Camera height above ground is known (user-configurable, default 1.5m)
- IMU pitch accuracy is 0.1–0.5 degrees

**Confidence (pitch-based):**
- Above 5°: 0.85 (target within ~17m)
- 2–5°: 0.70–0.85 (linear interpolation)
- 1–2°: 0.45–0.70 (increasing IMU noise ratio)
- 0.5–1°: 0.20–0.45 (significant uncertainty)
- Below 0.5°: fading to near-zero

**Slope risk penalty:** When pitch exceeds 3°, a multiplicative penalty is applied: `penalty = max(0.4, 1.0 - (pitch - 3.0) * 0.08)`. At 8.7° pitch, confidence drops from 0.85 to 0.46. This accounts for the fact that steep pitch angles usually indicate the user is looking downhill, not at a flat target 10m away.

**Confidence curve (distance-dependent fusion weight):**
- Below 5m: 0.0 (LiDAR is better)
- 5–10m: 0.0 → 0.70 (ramp)
- 10–50m: 0.70 (sweet spot)
- 50–100m: 0.70 → 0.45
- 100–200m: 0.45 → 0.20
- 200–500m: 0.20 → 0.05
- Beyond 500m: 0.0

### 2.4 DEM Ray-Casting (20–2000m)

Casts a ray from the device's GPS position through a digital elevation model (SRTM) and finds where the ray intersects terrain. This is the only source that can produce correct long-range estimates on sloped terrain.

**Algorithm:**
1. **Ray origin:** GPS latitude, longitude, and best available altitude (barometric preferred, GPS fallback)
2. **Ray direction:** true-north heading (from magnetometer) and pitch (from IMU), converted to East-North-Up (ENU) unit vector. Supports both downward-looking (negative pitch) and upward-looking rays (positive pitch up to +30°) for mountain/cliff scenarios where terrain rises above the observer.
3. **Ray marching:** step along ray in 30m increments (matching SRTM ~30m resolution), querying terrain elevation at each step
4. **Intersection detection:** when ray altitude drops below terrain elevation between two consecutive steps. For upward-looking rays, the ray marches forward and upward until it intersects rising terrain (mountain slope).
5. **Bisection refinement:** 5 iterations of binary search between the above/below steps, achieving ~1m accuracy
6. **Output:** horizontal great-circle distance to intersection point

**Elevation data:**
- **SRTM HGT tiles:** 3601 x 3601 signed Int16 big-endian, 1-arc-second (~30m) resolution, bilinear interpolation of 4 nearest posts
- **USGS EPQS fallback:** HTTP API providing 10m NED elevation (US coverage), cached on a 0.001° grid (~100m spacing)
- **LRU tile cache:** up to 3 tiles in memory (~75MB max)

**Confidence factors (multiplicative):**
- GPS accuracy: <5m → 1.0; 5–10m → 0.85; 10–15m → 0.65; ≥15m → 0.35
- Heading accuracy: <5° → 1.0; 5–10° → 0.80; 10–15° → 0.55; ≥15° → 0.35
- Distance factor: piecewise curve (see below)

**Lesson learned (GPS/heading penalties):** 1M Monte Carlo analysis showed poor GPS causes 12–31× more error at long range. The original factors (GPS: 0.85/0.70/0.40, heading: 0.80/0.60/0.40) were moderately tightened. The key insight: these factors affect *fusion weight* (not just displayed confidence), so overly aggressive penalties starve DEM of weight and increase error. The current values represent a balance — moderate fusion weight reduction, with a separate post-fusion DEM-only confidence penalty for display accuracy.

**Confidence curve (distance-dependent fusion weight):**
- Below 20m: 0.0 (GPS noise dominates)
- 20–40m: 0.0 → 0.75 (fast ramp — DEM viable)
- 40–100m: 0.75 → 0.92 (approaching sweet spot)
- 100–300m: 0.92 → 0.95 (DEM sweet spot)
- 300–600m: 0.95 → 0.88 (plateau, slight fade)
- 600–2000m: 0.88 → 0.65 (heading error grows, but ray-march averaging keeps accuracy high)
- Beyond 2000m: 0.0

**Lesson learned (DEM long-range confidence):** The original curve dropped DEM confidence from 0.88 → 0.50 at 600–2000m, which was too pessimistic. Monte Carlo testing showed DEM ray-casting achieves < 1% error even at 1500m thanks to ray-march step averaging. The flatter curve (0.88 → 0.65) matches actual measured accuracy.

**Rate limiting:** 2 Hz maximum. Results cached and reused when heading changes <1° and pitch changes <0.5°.

### 2.5 Object Detection Ranging (20–1000m+)

Detects known objects (people, vehicles, signs) in the camera image and computes range via the pinhole camera model using known real-world sizes.

**Pipeline:**
1. Object detection via YOLO model + Apple Vision framework at 5 FPS
2. Match detected objects against `KnownObjectDatabase` (ANSUR II anthropometric data, EPA vehicle dimensions, MUTCD sign specifications, USFWS wildlife data)
3. Pinhole model: `distance = (realSize x focalLength) / pixelSize`
4. Confidence = detection confidence x object size reliability x distance factor

**Confidence curve (distance-dependent):**
- Below 20m: 0.30 (bounding box imprecise relative to size)
- 20–50m: 0.70
- 50–200m: 0.85 (sweet spot)
- 200–500m: 0.80
- 500–1000m: 0.70
- Beyond 1000m: decaying from 0.70

---

## 3. Semantic Source Selection Engine

`UnifiedDepthField.swift` is the core of the system. It evaluates all available depth sources and selects ONE authoritative primary source per frame via the `semanticSelect()` priority chain, plus an independent background hypothesis.

### 3.1 SemanticSourceDecision Enum

```
lidarPrimary    — LiDAR selected (close range, <8m)
objectPrimary   — Object detection selected (known-size pinhole)
demPrimary      — DEM ray-cast selected (terrain target)
neuralPrimary   — Neural depth selected (<50m, calibrated)
geometricPrimary — Geometric ground-plane selected (fallback)
stadiametric    — User manual bracket input (highest priority)
none            — No valid source
```

### 3.2 Priority Chain

The `semanticSelect()` method evaluates sources in strict priority order. Each level is evaluated only if all higher-priority sources fail:

1. **Stadiametric** — If `stadiametricInput` is set with valid pixel size, return directly. This is user-explicit and overrides all sensor sources.

2. **LiDAR** — If LiDAR depth is valid, < 8m, and confidence is high. Authoritative for close range. Background: DEM estimate if available.

3. **Object Detection** — If a known-size object is detected at the crosshair with adequate confidence. Background: DEM estimate.

4. **DEM Ray-Cast** — If DEM estimate is available and no object is detected. The primary source for terrain targets (mountains, ridgelines). Background: neural or geometric estimate.

5. **Neural Depth** — If calibrated neural estimate is valid and < 50m (`AppConfiguration.neuralHardCapMeters`). The hard cap prevents unreliable extrapolation. Background: geometric estimate.

6. **Geometric Ground-Plane** — Fallback using `D = h / tan(pitch)`. No background (lowest priority).

### 3.3 Background Hypothesis

After selecting the primary source, `semanticSelect()` returns a secondary estimate from a different source type. The background provides the operator with alternate depth context:

- **Primary: LiDAR** → Background: DEM (foreground object vs. terrain behind)
- **Primary: Object** → Background: DEM
- **Primary: DEM** → Background: neural or geometric (terrain vs. foreground)
- **Primary: Neural** → Background: geometric

The background estimate is published via `@Published var backgroundDepth` and tracked by an independent Kalman filter in `RangingEngine`.

### 3.4 Confidence & Uncertainty

Confidence is computed per-source using `DepthSourceConfidence` curves, then normalized to account for the selected source's reliability at the measured distance.

**Uncertainty:** Computed from the selected source's error model:
- LiDAR: 2% of distance
- Neural: 5–10% depending on distance and calibration quality
- DEM: `sqrt(gpsAcc² + (5° × distance × π/180)²)`
- Object: based on bounding box pixel uncertainty
- Geometric: based on IMU pitch accuracy at the measured angle

### 3.5 Bimodal Detection

A log-scale histogram analyzer detects bimodal depth distributions (near foreground + far terrain):
- 30 bins spanning 3+ orders of magnitude
- Peaks separated ≥2× in distance, each >10% of ROI pixels
- Valley between peaks < 60% of smaller peak height
- When bimodal, `isBimodal` flag enables relaxed outlier thresholds in far-target mode

### 3.6 Scene Classification (Retained)

The depth-map scene classifier still operates at zero additional compute cost, informing future confidence decisions:
- **Sky detection:** zeros neural candidacy (meaningless max-depth)
- **Ground detection:** validates geometric model assumptions
- **Structure detection:** suppresses geometric (vertical surfaces violate flat-ground model)

---

## 4. Temporal Processing Pipeline

After the semantic selection engine produces per-frame primary and background estimates, dual temporal pipelines provide stability.

### 4.1 Dual Kalman Architecture

`RangingEngine` maintains two independent Kalman filters:

- **`fgKalmanFilter`** — tracks the primary (foreground) depth from `semanticSelect()`
- **`bgKalmanFilter`** — tracks the background hypothesis independently

**Source-switch reset:** When `semanticDecision` changes (e.g., NEURAL_PRIMARY → DEM_PRIMARY), the foreground Kalman filter is reset to prevent stale state contamination. The `previousSemanticDecision` is tracked for comparison. The background filter runs independently and is not affected by primary source switches.

### 4.2 Outlier Rejection (Ring Buffer Median)

A 5-frame ring buffer stores recent primary depth values. Each new measurement is compared against the buffer median. Distance-scaled deviation thresholds:

| Distance | Max Allowed Deviation |
|---|---|
| < 20m | 40% |
| 20–50m | 35% |
| 50–100m | 30% |
| > 100m | 25% |

Measurements exceeding the threshold are replaced by the buffer median. This prevents single-frame spikes from corrupting the Kalman filter state.

Exceptions:
- When the motion state is "panning", the raw measurement passes through
- **DEM-primary bypass:** When the semantic decision is `.demPrimary`, outlier rejection accepts the DEM reading directly without median comparison. On large transitions (>2× ratio), the ring buffer is cleared and the foreground Kalman filter is reset.
- **Far-target bimodal bypass:** When `targetPriority == .far` and bimodal detection is active, legitimate depth jumps >3× are allowed

### 4.3 Foreground Kalman Filter

A 1D Kalman filter with state vector `[depth, velocity]` where velocity = d(depth)/dt.

**State transition model (constant velocity):**
```
F = [1, dt]
    [0,  1]
```

**Process noise (motion-adaptive):**
- Stationary: 0.05 (strong prediction trust — user wants stable readout)
- Tracking: 0.50 (balanced)
- Panning: 2.50 (trust measurements — scene is changing)

**Measurement noise (confidence + distance dependent):**
```
R = R_base x (1/confidence) x distanceFactor
```
Where `distanceFactor` grows quadratically beyond 100m, reflecting the physics of inverse-depth calibration noise amplification.

**Between-frame prediction:** The filter's `predict()` method provides depth estimates between neural inference frames (67ms gaps at 15 FPS), filled by IMU-informed forward extrapolation.

### 4.3 Motion-Aware Smoother

An exponential moving average with adaptive alpha:
- Stationary: heavy smoothing (alpha ~0.15) for stable readout
- Tracking: moderate (alpha ~0.35)
- Panning: light smoothing (alpha ~0.70) for responsiveness

**Discontinuity detection:** When depth changes by more than 30% between frames, the smoother enters a confirmation period. After 3 consecutive frames confirm the new depth, it snaps to the new value. This prevents the EMA from slowly sliding during sudden scene changes (e.g., panning from a wall at 5m to a hillside at 200m).

---

## 5. Inclination Correction

After temporal filtering, inclination correction converts line-of-sight range to horizontal range:

```
adjustedRange = lineOfSightRange x cos(pitchAngle)
```

This matters for uphill/downhill shots where the bullet trajectory is governed by horizontal distance, not slant distance. The correction factor is displayed in the HUD.

For angles below 2° (0.035 rad), correction is skipped (factor = 1.0).

---

## 6. Ballistics Solver

Bullet drop and holdover computation using a step-integrated G1 point-mass trajectory solver with full Mach-dependent drag table.

**Trajectory solver (1-yard step integration):**
1. Convert target distance to yards (input always meters from fusion engine)
2. March trajectory in 1-yard increments:
   - Compute Mach number: `M = velocity / 1125` (speed of sound at sea level)
   - Look up G1 drag coefficient `Cd(M)` from 65-point Ingalls/Mayevski table with linear interpolation
   - Compute drag deceleration: `a = dragConstant × Cd(M) × v² / BC`
   - Update velocity via Euler integration, accumulate time of flight
3. Compute gravity drop: `drop = 0.5 × 32.174 × t²` (feet, converted to inches)
4. Compute sight-line holdover with proper geometry:
   ```
   holdover = drop(D) + h - (drop(Z) + h) × (D/Z)
   ```
   where `h` = sight height (1.5"), `D` = target distance, `Z` = zero distance
5. Convert to mils: `holdoverMils = holdoverInches / (targetYards × 0.036)`

**G1 drag table:** 65 (Mach, Cd) data points from the standard Ingalls/Mayevski G1 reference projectile. Captures the critical transonic drag peak at Mach 0.9–1.2 that dominates long-range trajectory. Binary search + linear interpolation for O(log n) lookup per step.

**Holdover display (multi-unit):** Shows mils (primary, for reticle reference), inches or centimeters (secondary, based on display unit setting), and direction (UP/DN). The HUD compact view shows `▲ 2.9 MIL UP · 52.0 IN`. The reticle overlay indicator shows an amber triangle with mil value and secondary unit.

**Reference accuracy (.308 168gr SMK, 100yd zero, G1 model vs published G7 data):**

| Distance | G1 Model | Published (G7) | G1 Over-prediction |
|---|---|---|---|
| 200 yd | ~4" / 0.6 mil | ~3.5" / 0.5 mil | +15% |
| 300 yd | ~15" / 1.4 mil | ~12.5" / 1.2 mil | +17% |
| 500 yd | ~68" / 3.8 mil | ~52" / 2.9 mil | +30% |
| 1000 yd | ~400" / 11.0 mil | ~370" / 10.3 mil | +8% |

G1 systematically over-predicts for boat-tail bullets (which better match G7 form factor). This is acceptable for a field holdover indicator — real-world conditions introduce larger variations.

Six caliber profiles with G1 ballistic coefficients, muzzle velocities, and bullet weights sourced from manufacturer data.

---

## 7. Operator Guidance Engine

`OperatorGuidanceEngine.swift` analyzes device motion via IMU gyroscope data to provide real-time coaching that improves ranging accuracy. Modeled on military marksmanship doctrine (FM 23-10, TC 3-22.10).

### 7.1 Stability Analysis

The engine maintains a rolling 30-sample window of angular velocity magnitudes and classifies stability from a 5-sample smoothed average:

| Stability Level | Angular Velocity (rad/s) | Description |
|---|---|---|
| Excellent | < 0.02 | Braced/tripod quality |
| Good | 0.02–0.05 | Steady handheld |
| Adequate | 0.05–0.10 | Tracking, moderate stability |
| Marginal | 0.10–0.30 | Moving, usable |
| Unstable | > 0.30 | Panning/shaking |

The stability percent (0.0–1.0) maps angular velocity linearly: 0.3 rad/s → 0%, 0.0 rad/s → 100%. This drives the stability bar fill level in the HUD.

### 7.2 Respiratory Pause Detection

Military doctrine teaches reading targets during the natural respiratory pause — a 2–3 second window at the bottom of each exhale cycle when the body is naturally still.

**Detection algorithm:**
1. Require at least 10 samples of angular velocity history
2. Compute mean of most recent 3 samples (current motion)
3. Compute mean of samples 3–8 ago (earlier motion)
4. If current < 0.02 rad/s AND earlier > 0.05 rad/s → **pause detected** (motion→stillness transition)
5. If current > 0.05 rad/s → pause cleared (motion resumed)

When a respiratory pause is detected during good+ stability, the "CAPTURE WINDOW" hint appears and the stability bar pulses green.

### 7.3 Reading Lock Detection

A "locked" reading confirms the displayed range has been stable long enough to trust:

1. Buffer last 60 range readings (~2–4 seconds at frame rate)
2. Filter to readings within the last 2 seconds (minimum 5 required)
3. Compute median of recent readings
4. If all readings are within 5% of the median → **reading locked**

### 7.4 Guidance Hints

The engine produces up to 14 distinct hints organized by priority (100 = highest) and severity (positive/caution/warning):

**Hint priority hierarchy:**
1. EXCESSIVE MOTION (100) — stability ≤ unstable
2. CALIBRATION NEEDED (90) — < 5 LiDAR calibration samples
3. HOLD STEADY (80) — marginal stability, < 5s duration
4. BRACE DEVICE (75) — marginal stability > 5s (prolonged instability)
5. LOW LIGHT (70) — ambient light < 15%
6. GPS ACQUIRING (65) — no GPS fix or not authorized
7. GPS LOW ACCURACY (60) — horizontal accuracy > 20m
8. COMPASS INTERFERENCE (55) — heading accuracy > 15°
9. CALIBRATION STALE (50) — calibration age > 120s + confidence < 50%
10. CALIBRATING (40) — samples 1–19, age < 30s (active calibration in progress)
11. MULTIPLE READINGS (30) — range > 200m, not locked, adequate+ stability
12. CAPTURE WINDOW (25) — respiratory pause detected during good+ stability
13. READING LOCKED (20) — range stable 2+ seconds during adequate+ stability
14. STABILIZED (10) — good+ stability for 1–4s after transition from unstable

**Display rules:**
- Maximum 2 hints visible simultaneously
- Hints sorted by priority, top 2 shown
- Minimum 2-second display duration prevents flicker
- Existing hints within their minimum duration are retained even if lower-priority

### 7.5 Integration with AppState

`AppState.updateGuidanceEngine(timestamp:)` runs every frame (~30 Hz) and feeds:
- Angular velocity from `InclinationManager`
- Current range (when valid) for reading-lock detection
- GPS fix status and accuracy from `LocationManager`
- Calibration age, confidence, and sample count from `ContinuousCalibrator`
- Heading accuracy (estimated ~10° from CMMotionManager reference frame)

---

## 8. Reticle System

### 8.1 Reticle Styles

Three configurable reticle styles rendered in First Focal Plane (FFP):

**MIL-DOT (NATO standard):**
- Duplex crosshair arms (thick outer → thin inner)
- 1-mil interval dots (filled or hollow, configurable)
- Optional half-mil hash marks
- Optional mil number labels
- Center dot

**CROSSHAIR (maximum clarity):**
- Duplex crosshair arms only
- Center dot
- No dots, hashes, or labels — designed for digital-only ranging

**RANGEFINDER (Vectronix VECTOR style):**
- Duplex crosshair arms
- L-shaped corner brackets forming a 2×2 mil square around center
- Bracket arm length: 0.4 mil
- Center dot
- Designed for quick angular size reference and target framing

### 8.2 Depth Zone Brackets (Always-On)

All reticle styles include an always-on depth zone bracket overlay that visualizes the relationship between the crosshair/fusion reading and the DEM terrain reading:

**Inner bracket:** Shows the crosshair depth (fusion result or neural/LiDAR foreground reading). Normally green; turns amber when in disagreement with DEM.

**Outer bracket:** Shows the DEM terrain distance. Normally green; turns cyan when in disagreement with the inner bracket.

**Disagreement detection:** When the ratio between crosshair depth and DEM depth exceeds 2.0×, the brackets change color to signal that the fusion is reading a foreground object while DEM sees further terrain behind it. This gives the operator visual confirmation that terrain routing is active (outer bracket = DEM distance) vs. when the crosshair is reading a near object (inner bracket = object distance).

**Configuration:** The `DepthZoneOverlay` struct in `ReticleConfiguration.swift` computes `isActive` (any valid depth reading), `hasDisagreement` (>2× ratio), and provides `formatDepth()` for bracket labels.

### 8.3 FFP Scaling

All reticle elements scale proportionally with camera zoom factor, maintaining calibrated angular measurements at any magnification. The `ReticleGeometry` module converts between mils and pixels using:

```
pixelsPerMil = (imageWidth / (2 x tan(HFOV/2))) x (PI/3200) x zoomFactor
```

### 8.4 Configuration

`ReticleConfiguration` stores: style, color, line widths, opacity, dot fill, hash marks, mil labels, and outline. Settings persisted via `@Published` bindings in `AppState`. Color presets: NVG (phosphor green), DAY (amber), RED.

---

## 9. Compass Bearing Display

The HUD displays a compass bearing chip showing true-north heading with cardinal direction indicator (e.g., "045°NE"). The heading is derived from `InclinationManager` using `CMAttitudeReferenceFrame.xTrueNorthZVertical`, which fuses magnetometer + gyroscope + accelerometer data for stable heading.

The bearing chip is visible only when GPS is active (since magnetometer calibration benefits from location data). Cardinal direction is computed from 8-point sectors (N, NE, E, SE, S, SW, W, NW) using 45° arcs centered on each cardinal.

---

## 10. Barometric Altimeter Integration (unchanged)

`LocationManager` combines GPS altitude with CMAltimeter barometric data:

**Sources:**
- `CMAltimeter.startAbsoluteAltitudeUpdates()` — fused GPS+barometer altitude with 1–5m vertical accuracy
- `CMAltimeter.startRelativeAltitudeUpdates()` — pressure-based altitude changes with ~0.1m precision
- `CLLocation.altitude` — GPS-only altitude with 10–30m vertical accuracy

**Source selection (`updateBestAltitude()`):**
When both barometric and GPS altitude are valid, the source with better vertical accuracy is selected. The selected source propagates to `bestAltitude` and `bestVerticalAccuracy`, which the DEM estimator uses for ray origin altitude and confidence computation.

**Impact on DEM confidence:**
- Barometric (<3m accuracy): altFactor = 1.0
- Good GPS (<10m): altFactor = 0.85
- Typical GPS (<20m): altFactor = 0.70
- Poor/unknown: altFactor = 0.60

---

## 11. Sensor Coordinate Frames

**InclinationManager:**
- Reference frame: `.xTrueNorthZVertical` (CMAttitudeReferenceFrame)
- Pitch: rotation about device X-axis. 0 = level, negative = looking down, positive = looking up
- Heading: derived from attitude yaw, mapped to 0–360° true north
- Update rate: 60 Hz (CMMotionManager)

**LocationManager:**
- Coordinate: WGS84 (latitude, longitude)
- Altitude: meters above WGS84 ellipsoid (barometric or GPS)
- Accuracy: meters (horizontal and vertical)

**Camera:**
- ARKit provides intrinsic matrix (focal length fx, fy in pixels)
- Camera pose from ARFrame.camera.transform (4x4 matrix)
- Depth maps aligned to camera image coordinates

---

## 12. Logging

Twelve `os.Logger` categories provide structured, filterable diagnostics:

| Category | Subsystem |
|---|---|
| Camera | Camera capture, zoom, lens switching |
| Depth | LiDAR sampling, depth map processing |
| Neural | CoreML inference, model loading |
| Ranging | Pipeline stages, fused output |
| Fusion | Source weights, disagreement, outlier suppression |
| Calibration | LiDAR-neural pairs, fitting, quality |
| Reticle | Mil-dot geometry, rendering |
| Sensors | IMU, barometer, motion state |
| UI | View lifecycle, user interactions |
| Detection | Object detection, YOLO results |
| Performance | FPS, inference timing, thermal |
| Terrain | SRTM tiles, EPQS queries, DEM results |

All logging uses Apple's unified logging system (`os.Logger`) for zero-cost disabled logging and Console.app filtering.

---

## 13. Concurrency Model

The app uses Swift 6 strict concurrency throughout:

- **`@MainActor`**: all UI state (`AppState`, `RangingEngine`, `OperatorGuidanceEngine`, view models, `SceneClassifier`)
- **`actor`**: `SRTMTileCache` (thread-safe elevation queries)
- **`@unchecked Sendable`**: `ContinuousCalibrator` (NSLock-protected internal state)
- **Structured concurrency**: `async/await` for neural inference, DEM queries, and object detection
- **Task management**: ring buffer pattern prevents CoreML task stacking (only one inference in flight at a time)

Frame processing uses a `processingInFlight` flag to drop frames when the pipeline is busy, maintaining responsiveness without task pile-up.

---

## 14. Stadiametric Manual Ranging

### 14.1 Pinhole Formula

When STADIA mode is active, the user adjusts a draggable bracket overlay to span a target of known size. The range is computed via the pinhole camera model:

```
R = (knownSize × focalLength) / pixelSize
```

Where:
- `knownSize` = real-world target height in meters (user-configurable)
- `focalLength` = camera focal length in pixels (~2160px for iPhone main camera)
- `pixelSize` = pixel distance between top and bottom brackets

### 14.2 Target Presets

| Preset | Height (m) | Use Case |
|---|---|---|
| PERSON | 1.8 | Standing human (ANSUR II 50th percentile) |
| VEHICLE | 1.5 | Standard sedan/SUV roof height |
| DEER | 1.0 | White-tail deer shoulder height |
| DOOR | 2.0 | Standard doorway |
| FENCE POST | 1.2 | Typical fence post |
| POWER POLE | 10.0 | Utility pole |
| WINDOW | 1.0 | Standard window height |

### 14.3 UI Implementation

The `StadiametricBracketOverlay` provides two draggable horizontal amber lines with:
- Center drag handles (30×6pt) for precise positioning
- Dashed connecting line between brackets
- Pixel distance label showing bracket span
- 30pt hit target zones for easy touch interaction

The bracket pixel distance feeds into `AppState.updateStadiametricInput()`, which creates a `StadiametricInput` struct and feeds it to `UnifiedDepthField.stadiametricInput`. Since stadiametric has the highest semantic priority, it overrides all sensor-based sources.

---

## 15. DEM Map Verification (MapPiP)

When MAP mode is enabled and GPS is active, a 140×100pt satellite map overlay shows:
- **User location** — blue dot at current GPS position
- **DEM hit point** — red pin at the ray-cast terrain intersection coordinate
- **Ray polyline** — amber line connecting user to hit point
- **Satellite imagery** — MapKit `.imagery(elevation: .realistic)` style

The MapPiP provides visual confirmation that the DEM ray-cast is hitting the expected terrain feature. Camera position auto-centers to fit both points with appropriate padding.

---

## 16. Monte Carlo Fusion Validation

The semantic selection algorithm is validated by a deterministic Monte Carlo simulator (`MonteCarloFusionTests.swift`) that replicates the `UnifiedDepthField` semantic selection logic with synthetic sensor noise.

### 14.1 Test Architecture

**Seeded RNG:** xorshift128+ with fixed seed (42) ensures bit-identical runs across platforms and CI.

**Sensor Noise Model:** Each source receives distance-dependent Gaussian noise calibrated to real-world sensor characteristics:
- LiDAR: ±2% at sweet spot, ±5% at range limits
- Neural: ±8% within calibration range (0.2–8m), ±25% extrapolated
- Geometric: ±15% base, ×2 penalty when pitch < 3° (near-flat ground)
- DEM: ±5% base, scaled by GPS accuracy (1× at 5m, 3× at 15m+) and heading accuracy
- Object: ±10% base

**Fusion Simulator:** Mirrors production `UnifiedDepthField` semantic selection logic including:
- **Semantic source selection priority chain** (Stadia > LiDAR > Object > DEM > Neural > Geometric)
- **Neural hard cap at 50m** (confidence = 0.0 beyond `neuralHardCapMeters`)
- Distance-dependent confidence curves from `DepthSourceConfidence`
- DEM-dominance rule (ratio > 1.5×, DEM > 40m → suppress neural)
- Neural extrapolation penalty (beyond 15m calibration range)
- Confidence computation per selected source
- DEM-only single-source confidence penalty (×0.85 at >100m)

### 14.2 Two-Tier Testing

**Tier 1 — CI smoke test (10K samples, ~2 seconds):**
- 1,000 samples per band across 10 distance bands (1–2000m)
- Pass criteria: global mean error < 10%, zero catastrophic (>100%) errors
- Runs on every build

**Tier 2 — Deep analysis (1M samples, ~22 seconds):**
- 100,000 samples per band
- Uses `StreamingStats` histogram-based percentile estimation (500 bins, O(1) memory per category)
- Reports P50/P90/P95/P99 per band, source combination analysis, slope effects, GPS impact
- Used for tuning confidence curves and fusion parameters

### 14.3 Results (10K Samples with Terrain Routing, Current Tuning)

With terrain routing active, the fusion simulator now short-circuits to DEM for terrain targets (no detected object), matching the production pipeline. The 10K CI test results:

| Band | Range | Mean Error | Max Error |
|---|---|---|---|
| 1 | 0.3–3m | 0.4% | 1.5% |
| 2 | 3–8m | 1.1% | 5.0% |
| 3 | 8–15m | 2.6% | 11.9% |
| 4 | 15–30m | 6.3% | 33.6% |
| 5 | 30–50m | 6.2% | 67.7% |
| 6 | 50–100m | 4.7% | 84.9% |
| 7 | 100–200m | 4.7% | 90.5% |
| 8 | 200–500m | 6.3% | 94.4% |
| 9 | 500–1000m | 5.9% | 95.9% |
| 10 | 1000–2000m | 7.2% | 71.4% |

**Global:** 5.2% mean error, 0 catastrophic (>100%), 0.91% confident-and-bad (>50% error with >0.3 confidence)

**Source breakdown at long range (terrain routing active):**
- DEM+Object available: 2.6–4.6% mean error (object corroborates DEM)
- DEM-only (terrain routing): 6.7–11.0% mean error (DEM as sole authority)
- No DEM: 0 samples at 30m+ when GPS is available (terrain routing catches all)

### 14.4 Key Findings

1. **Terrain routing eliminates the foreground obstruction problem.** The original 5-source weighted average could not produce correct terrain distance when 3–4 sources saw a foreground obstacle (rock wall at 2–9m) and only DEM saw terrain (1600m). Terrain routing short-circuits this by treating DEM as the authoritative answer when no discrete object is detected.

2. **GPS quality is the dominant long-range accuracy predictor.** Poor GPS (>15m accuracy) causes 12–31× more error at 500m+ than good GPS (<5m). The fusion weight penalty is moderate (0.35) because aggressive penalties starve DEM of weight entirely; the remaining error is flagged via lower displayed confidence.

3. **DEM + Object corroboration is 3–4× more accurate than DEM-only.** When object detection confirms DEM range, the agreement bonus and mutual validation produce significantly better results. A 15% confidence penalty is applied when DEM is the sole source at >100m.

4. **DEM pitch guard must allow upward-looking rays.** The original guard (`pitch < -0.17°`, downward only) blocked DEM when looking at mountains above the observer's horizon. The ray-march math handles upward rays correctly — the ray marches forward and upward until it intersects rising terrain. The guard was relaxed to allow rays up to +30° above horizontal.

5. **Neural extrapolation degrades gracefully.** The steepened extrapolation penalty (0.9 → 0.10 over 15–100m) prevents neural from dominating fusion at ranges where its calibration is invalid, while DEM-dominance suppression (neural weight ×0.38 when DEM disagrees by >1.5×) provides a second safety net.

---

## 17. Future Work

### 15.1 Near-Term Refinements

- **Geometric confidence at gentle slopes:** Add steeper pitch-based penalty for 2–5° angles at 15–30m range (currently the weakest band at 6.2% mean error)
- **GPS-aware displayed confidence:** Post-fusion confidence modifier that further reduces displayed confidence when GPS accuracy >10m, separate from the fusion weight factor
- **Dynamic BDC marks:** Scale mil-dot subtension markers on the reticle to match the ballistic drop curve for the selected cartridge

### 15.2 Medium-Term Features

- **LASE button:** Tap-to-lock that freezes the current range reading for 5 seconds, useful for unstable hold positions
- **Guidance engine unit tests:** Formal test coverage for `OperatorGuidanceEngine` stability analysis, respiratory pause detection, and reading lock logic
- **Multi-target tracking:** Maintain range estimates for multiple crosshair positions simultaneously
- **Wind estimation integration:** Incorporate wind speed/direction for ballistic solver lateral drift

### 15.3 Long-Term Research

- **Field validation dataset:** Collect ground-truth range measurements at known distances (laser rangefinder reference) across varied terrain to validate Monte Carlo predictions against real-world performance
- **Adaptive confidence curves:** Use field data to auto-tune confidence curve parameters per-device and per-environment
- **Collaborative elevation data:** Crowdsource high-resolution elevation corrections to improve DEM accuracy beyond SRTM 30m resolution
- **Thermal/IR sensor fusion:** Extend depth source framework to support thermal imaging for low-visibility conditions
