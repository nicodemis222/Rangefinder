# Rangefinder — Technical Whitepaper

## Semantic Source Selection for Mobile Rangefinding: From LiDAR to Terrain Ray-Casting

---

## Abstract

We present a real-time depth estimation system for iOS that selects among six heterogeneous depth sources — LiDAR time-of-flight, neural monocular depth estimation, geometric ground-plane trigonometry, digital elevation model (DEM) ray-casting, object-size pinhole ranging, and user-directed stadiametric bracket ranging — using a priority-based semantic source selection state machine with multi-hypothesis tracking from 0.3 to 2000 meters. The system addresses the fundamental challenge that no single depth source covers the full operational range with adequate accuracy, and that averaging sources measuring fundamentally different things (foreground rock wall vs. background mountain) produces catastrophically wrong results. Our approach replaces weighted-average fusion with a deterministic priority chain: Stadiametric > LiDAR (< 8m, with foreground occluder exception) > Object Detection > DEM > Neural (< 150m hard cap) > Geometric. Dual Kalman filters independently track foreground and background depth hypotheses, with automatic filter reset on semantic source switches. A neural depth hard cap at 150m prevents unreliable inverse-depth extrapolation while allowing neural to contribute at medium ranges. Comprehensive testing — Monte Carlo simulation across 10 distance bands (0.3–2000m), distance sweep validation (100–1500 yards) with multi-operator repeatability and environmental condition matrices, and a 10,000-sample ground truth dataset from ARKitScenes and DIODE with laser-scanner reference measurements — demonstrates mean error below 10% at all distances with 312 unit tests across 24 test files. Golf-specific stadiametric ranging with a USGA regulation flagstick preset (2.13m) enables club-selection grade accuracy (±2 yards at 150) at 5–8× zoom. An IMU-based operator guidance engine provides real-time coaching (stability detection, respiratory pause capture windows, reading lock confirmation) modeled on military marksmanship doctrine (FM 23-10).

---

## 1. Introduction

### 1.1 Problem Statement

Accurate range estimation from a mobile device across 0–2000m faces a coverage gap problem. Each available depth sensing modality has a limited effective range and specific failure modes:

| Source | Effective Range | Primary Failure Mode |
|---|---|---|
| LiDAR (dToF) | 0.3–12m | Signal attenuation beyond 5m |
| Neural depth (calibrated) | 2–150m | Inverse-depth noise amplification |
| Geometric (ground plane) | 5–200m | Assumes flat terrain |
| DEM ray-casting | 20–2000m | Requires GPS fix, heading accuracy |
| Object detection | 20–1000m | Requires recognized objects in view |

No weighted average of sources that individually produce wrong answers can produce a right answer. Averaging sources that measure fundamentally different things — a rock wall at 2m (LiDAR) vs. a mountain at 1600m (DEM) — produces a catastrophically wrong 25m estimate regardless of confidence tuning.

### 1.2 Approach

We replace the weighted-average fusion architecture with a **semantic source selection** state machine. Instead of blending all sources, the system selects ONE authoritative primary source per frame via a deterministic priority chain:

```
Priority: Stadiametric > LiDAR (<12m) > Object > DEM > Neural (<150m) > Geometric
```

A secondary "background hypothesis" from a different source is tracked independently via dual Kalman filters, providing the operator with context about alternate depth readings (e.g., foreground obstacle vs. terrain behind it). Key architectural decisions:

1. **Neural hard cap at 150m** — inverse-depth noise amplification makes neural estimates unreliable beyond 150m; confidence drops to zero. The calibrator's compression cap was removed to allow natural extrapolation up to this boundary.
2. **Dual Kalman filters** — foreground and background tracked independently; filter resets on source switches
3. **Stadiametric manual ranging** — user-directed bracket overlay using pinhole formula `R = (knownSize × focalLength) / pixelSize`, with 8 target presets including USGA regulation golf pin (2.13m)
4. **DEM map verification** — MapKit picture-in-picture showing ray-cast hit point on satellite imagery

---

## 2. Mathematical Foundations

### 2.1 Geometric Ground-Plane Ranging

Given a camera at height *h* above a flat ground plane, tilted downward by angle *theta* below horizontal:

```
D = h / tan(theta)
```

**Error propagation:** The relative error in *D* is dominated by IMU pitch noise *delta_theta*:

```
dD/D = delta_theta / (tan(theta) * theta)    (for small theta)
```

More precisely:

```
dD = h * delta_theta / sin^2(theta)
dD/D = delta_theta / (sin(theta) * cos(theta)) = delta_theta / (0.5 * sin(2*theta))
```

For iPhone IMU accuracy of delta_theta approximately 0.1 degrees (0.00175 rad):

| Pitch (deg) | Distance (m) at h=1.5m | Relative Error | Absolute Error |
|---|---|---|---|
| 8.53 | 10.0 | 1.2% | 0.12m |
| 1.72 | 50.0 | 5.8% | 2.9m |
| 0.86 | 100.0 | 11.6% | 11.6m |
| 0.43 | 200.0 | 23.3% | 46.6m |
| 0.17 | 500.0 | 58.8% | 294m |

The error grows as `1/sin(2*theta)`, making geometric ranging increasingly unreliable beyond 100m.

**Slope risk:** At 8.7 degrees pitch with h = 1.5m, the geometric model computes D = 9.8m. If the user is actually on a hillside looking at a target 91m away, the error is 830%. The slope penalty factor `max(0.4, 1.0 - (theta - 3) * 0.08)` at 8.7 degrees gives 0.544, reducing geometric weight to prevent it from corrupting the fusion when DEM provides the correct answer.

### 2.2 Neural Depth Calibration (Inverse Depth Transform)

DepthAnythingV2 outputs inverse/relative depth: close objects produce high values, far objects produce low values. The relationship between neural output *n* and metric depth *d* is:

```
d = scale / n + shift
```

This is fit via weighted least-squares regression on pairs `(1/n_i, d_lidar_i)` collected in the 0.2–8m LiDAR overlap zone. The weights incorporate exponential time decay:

```
w_i = decay^(t_current - t_i) * confidence_i
```

With decay = 0.95 per second and a rolling window of 50 samples.

**Noise amplification at long range:** The inverse transform amplifies noise quadratically with distance. If the neural model has disparity noise sigma_n:

```
sigma_d = scale * sigma_n / n^2 = (d^2 / scale) * sigma_n
```

At 50m with scale approximately 10: sigma_d = 250 * sigma_n. At 200m: sigma_d = 4000 * sigma_n. This quadratic growth motivates the neural hard cap at 150m (`AppConfiguration.neuralHardCapMeters = 150.0`) — beyond this distance, neural confidence drops to exactly zero and the source is excluded from semantic selection. The 150m boundary (extended from 50m in the pipeline overhaul) was chosen to allow neural to contribute useful signal at medium ranges where confidence is low but non-zero, while the calibrator's compression cap was removed to allow natural extrapolation.

**Extrapolation risk:** Calibration is trained on 0.2–8m data. Beyond 8m, the affine model is extrapolating. The calibration quality modifier provides a 30-second full-confidence window after the last LiDAR pair, then decays to a floor of 0.30 over 5 minutes.

### 2.3 DEM Ray-Terrain Intersection

**Coordinate system:** The ray is defined in an East-North-Up (ENU) coordinate frame centered at the device's GPS position.

**Ray parameterization:** Given device altitude *A*, pitch below horizontal *phi*, and heading *psi* (true north clockwise):

```
ray(t) = origin + t * direction

origin = (0, 0, A)

direction = (sin(psi) * cos(phi),    // East component
             cos(psi) * cos(phi),     // North component
             -sin(phi))                // Up component (negative = downward, positive = upward)
```

Where *t* is distance along the ray in meters. The Up component is positive when the observer looks upward (negative *phi*), causing the ray to march forward and upward. This is essential for mountain/cliff scenarios where terrain rises above the observer's position.

**Pitch acceptance range:** The ray-cast accepts device pitch from steep downward up to +30 degrees above horizontal. Rays steeper than +30 degrees are rejected (aiming at sky, unlikely to intersect terrain). The ray-march math handles upward rays correctly — when the ray marches upward into rising terrain, the intersection detection fires when terrain elevation exceeds ray altitude.

**ENU to geographic conversion:**

```
lat(t) = lat_0 + northM(t) / 111320
lon(t) = lon_0 + eastM(t) / (111320 * cos(lat_0))
```

**Intersection detection:** At each step *t_k = k * 30m*, the ray altitude is compared with terrain elevation:

```
rayAlt(t_k) = A + t_k * (-sin(phi))
terrainElev(t_k) = SRTM_bilinear(lat(t_k), lon(t_k))

if rayAlt(t_{k-1}) > terrainElev(t_{k-1}) AND rayAlt(t_k) <= terrainElev(t_k):
    intersection found between t_{k-1} and t_k
```

**Bisection refinement:** 5 iterations of binary search between the above/below step:

```
for i in 1..5:
    t_mid = (t_lo + t_hi) / 2
    if rayAlt(t_mid) > terrainElev(t_mid):
        t_lo = t_mid
    else:
        t_hi = t_mid

result = (t_lo + t_hi) / 2    // approximately 30m / 2^5 = 0.9m accuracy
```

**Bilinear interpolation of SRTM data:**

SRTM tiles provide elevation at discrete 1-arc-second posts (~30m spacing). For a query point between posts:

```
elev = e00 * (1-fx)(1-fy) + e01 * fx(1-fy) + e10 * (1-fx)fy + e11 * fx*fy
```

Where fx, fy are fractional grid positions and e00, e01, e10, e11 are the four corner elevations.

**Error sources:**
- GPS horizontal accuracy sigma_gps: shifts the ray origin, causing lateral error that grows with distance
- Heading accuracy sigma_heading: at distance *d*, lateral error = d * tan(sigma_heading)
- Altitude accuracy sigma_alt: shifts ray vertically, causing range error proportional to 1/sin(phi)
- SRTM resolution: 30m grid cannot resolve features smaller than 30m

**Heading error example:**
At 500m with 5 degree heading error: lateral displacement = 500 * tan(5 degrees) = 43.7m. On a hillside with 10 degree slope, this lateral error translates to roughly 7.7m range error — still dramatically better than geometric ranging would produce on the same terrain.

### 2.4 Object-Size Pinhole Ranging

Given an object of known real-world size *S* (meters), observed with apparent pixel size *p* on a camera with focal length *f* (pixels):

```
D = (S * f) / p
```

**Error analysis:** If bounding box error is delta_p pixels:

```
dD/D = delta_p / p
```

At 100m, a 1.7m person (ANSUR II 50th percentile male) on a 4032px-wide image at 78 degree HFOV:

```
f = 4032 / (2 * tan(39deg)) = 2484 pixels
p = (1.7 * 2484) / 100 = 42.2 pixels
```

A 2-pixel bounding box error gives dD/D = 4.7% or 4.7m — competitive with DEM at this range and independent of terrain slope.

### 2.5 Kalman Filter

**State vector:** `x = [depth, velocity]^T`

**State transition:**
```
F = [1  dt]
    [0   1]
```

**Observation model:** `H = [1  0]` (we observe depth directly)

**Predict step:**
```
x_predicted = F * x
P_predicted = F * P * F^T + Q
```

Where Q is the process noise matrix:
```
Q = q * [dt^3/3  dt^2/2]
        [dt^2/2  dt    ]
```

And *q* is motion-adaptive: 0.05 (stationary), 0.50 (tracking), 2.50 (panning).

**Update step:**
```
y = z - H * x_predicted                    // innovation
S = H * P_predicted * H^T + R              // innovation covariance
K = P_predicted * H^T * S^{-1}             // Kalman gain
x_updated = x_predicted + K * y
P_updated = (I - K * H) * P_predicted
```

**Measurement noise R:** Adapts to confidence and distance:
```
R = R_base * (1/confidence) * distanceFactor(depth)
```

Where `distanceFactor = min(25, (d/50)²)` grows quadratically but is capped at 25 to keep the filter responsive at long range. Without the cap, extreme distances (500m+) would produce distanceFactor > 100, making Kalman gain effectively zero and the filter unresponsive to valid new measurements. The cap of 25 provides a balance: at 500m, R = R_base × (1/confidence) × 25, giving Kalman gain ≈ 0.1 — the filter still tracks new measurements while maintaining stability.

---

## 3. Semantic Source Selection Architecture

### 3.1 Design Rationale

The original MilDot Rangefinder used weighted-average fusion across all sources. This fails catastrophically when sources measure fundamentally different things: LiDAR sees a rock wall at 2m, DEM sees terrain at 1600m, and the weighted average produces ~25m — wrong by 64×. No amount of confidence curve tuning can make an average of 2m and 1600m equal 1600m.

The replacement architecture uses **semantic source selection**: a priority-based state machine (`semanticSelect()`) that picks ONE authoritative primary source per frame, plus an independent background hypothesis from a different source.

### 3.2 Priority Chain (SemanticSourceDecision)

The `semanticSelect()` method in `UnifiedDepthField` evaluates sources in strict priority order:

```
1. STADIAMETRIC  — User-explicit manual bracket (highest priority)
2. LIDAR_PRIMARY — Close range <8m, high confidence
   EXCEPTION: skipped when foreground occluder detected (§3.8.1)
3. OBJECT_PRIMARY — Known-size pinhole ranging at crosshair
4. DEM_PRIMARY   — Terrain target (no object detected)
5. NEURAL_PRIMARY — Calibrated depth, <150m hard cap
6. GEO_PRIMARY   — Ground-plane fallback
7. NONE          — No valid source available
```

Each level is evaluated only if all higher-priority sources fail their gating conditions. The selected decision is published as `semanticDecision` and displayed in the HUD.

### 3.3 Background Hypothesis

After selecting the primary source, `semanticSelect()` returns a secondary "background" estimate from a different source type. This provides the operator with alternate depth context:

- **LiDAR primary** → DEM as background (foreground object vs. terrain behind)
- **Object primary** → DEM as background
- **DEM primary** → Neural or geometric as background (terrain vs. foreground); **LiDAR when foreground occluder detected** (shows "BG LIDAR Xm" for foreground context)
- **Neural primary** → Geometric as background

The background estimate is tracked by an independent Kalman filter (`bgKalmanFilter`) and displayed as a "BG" chip below the primary range readout.

### 3.4 Neural Hard Cap (150m)

Neural depth estimation uses inverse-depth calibration that amplifies noise quadratically with distance. Beyond 150m, the error growth makes neural estimates unreliable. The hard cap is enforced at two levels:

1. **DepthSourceConfidence.neural()**: returns 0.0 for distances >= 150m
2. **semanticSelect()**: skips neural candidate when estimated distance >= `AppConfiguration.neuralHardCapMeters`

The 150m threshold (extended from 50m in the pipeline overhaul) allows neural to contribute useful signal at medium ranges with progressively lower confidence. At 150m with scale ~10, sigma_d = 2250 × sigma_n, making neural estimates unreliable. The calibrator's compression cap was removed to allow natural extrapolation up to 150m without artificially squashing distances.

### 3.5 Stadiametric Manual Ranging

When the user activates STADIA mode, a draggable bracket overlay appears on screen. The user aligns top and bottom brackets with a target of known size. The pinhole formula computes range:

```
R = (knownSize × focalLength) / pixelSize
```

Where `knownSize` is configurable (8 presets: PERSON 1.8m, VEHICLE 1.5m, DEER 1.0m, DOOR 2.0m, FENCE POST 1.2m, POWER POLE 10.0m, WINDOW 1.0m, GOLF PIN 2.13m) and `pixelSize` is the pixel distance between brackets. Stadiametric input has the highest priority in semantic selection — it overrides all sensor-based sources. The GOLF PIN preset uses the USGA regulation flagstick height (7 ft / 2.13m) and is specifically designed for golf rangefinding at 40–300 yards with 5–8× telephoto zoom.

### 3.6 Dual Kalman Filters

`RangingEngine` maintains two independent Kalman filters:

- **`fgKalmanFilter`** — tracks the primary (foreground) depth estimate
- **`bgKalmanFilter`** — tracks the background hypothesis

When the `semanticDecision` changes (e.g., from NEURAL_PRIMARY to DEM_PRIMARY), the foreground Kalman filter is reset to prevent stale state contamination. The background filter runs independently and is not affected by primary source switches.

### 3.7 Scene Classification (Retained)

The depth-map scene classifier still operates at zero additional compute cost, analyzing the neural depth map for sky/ground/structure detection. In the semantic architecture, it informs future confidence decisions but does not directly modify source weights (the priority chain handles source selection).

### 3.8 Bimodal Detection

The system detects bimodal depth distributions (near foreground + far terrain) using log-scale histogram analysis. When bimodal conditions are detected (peaks separated ≥2× in distance, each covering >10% of ROI pixels), the `isBimodal` flag is published and the target priority chip shows an amber indicator. In far-target mode with bimodal detection, outlier rejection thresholds are relaxed to allow legitimate depth jumps.

#### 3.8.1 Foreground Occluder Detection

When aiming OVER nearby objects (rocks at 3m) at distant terrain (~1600m), LiDAR reads the close foreground and would normally win the priority chain unconditionally. The `isForegroundOccluder()` predicate detects this by checking four conditions (ALL required):

1. `targetPriority == .far` — user wants the distant target
2. `bimodal.isBimodal` — scene has two distinct depth populations
3. `bimodal.demAgreesWithFar` — DEM corroborates the far peak (within 30%)
4. LiDAR depth is in the near cluster (nearPeak < 12m, or lidarDepth ≤ nearPeak)

When triggered, LiDAR is skipped and DEM becomes primary. LiDAR is demoted to the background entry ("BG LIDAR 3m"), giving the user foreground context. This is a wiring fix — no new algorithm; the bimodal analysis and DEM agreement signals were already computed but not consulted in the LiDAR selection block.

| Scenario | Bimodal? | DEM Agrees? | Priority | Result |
|---|---|---|---|---|
| Rocks 3m + mountains 1600m | Yes | Yes | .far | DEM primary (fix) |
| Same scene | Yes | Yes | .near | LiDAR primary (user wants near) |
| Indoor room, no DEM | No | N/A | .far | LiDAR primary (safe) |
| Single object at 5m | No | N/A | .far | LiDAR primary (safe) |
| Bimodal, DEM disagrees | Yes | No | .far | LiDAR primary (safe) |

---

## 4. Failure Mode Analysis

### 4.1 Case Study: 100 Yards on Sloped Terrain

**Scenario:** User stands on a hilltop, looking down at 8.7 degrees pitch at a target 91.4m (100 yards) away.

**Without DEM (pre-terrain integration):**

| Source | Estimate | Weight | Contribution |
|---|---|---|---|
| Geometric | 9.8m | 0.57 | 5.6 |
| Neural (calibrated) | 35m | 0.77 | 27.0 |
| **Fused** | **24.3m (27 YDS)** | **1.34** | — |

Both sources are catastrophically wrong. No confidence tuning can fix this — a weighted average of 9.8m and 35m cannot produce 91m.

**With DEM + scene routing + outlier suppression:**

| Source | Estimate | Weight | Notes |
|---|---|---|---|
| Geometric | 9.8m | 0.31 → **0** | Slope-penalized, then suppressed as outlier |
| Neural | 35m | 0.50 | Decayed at 35m distance |
| DEM | 91m | 0.55 | Good GPS, sweet spot distance |
| **Fused** | **67.3m (74 YDS)** | **1.05** | — |

With stale neural calibration (lower quality), DEM dominates further, approaching the correct 91m.

### 4.2 Failure: No GPS Signal

When GPS is unavailable (indoors, urban canyon):
- DEM: not available (no ray origin)
- Geometric: slope-penalized at steep pitch → low weight
- Neural: sole contributor with decayed confidence
- Result: low displayed confidence honestly signals uncertainty

### 4.3 Failure: Pointing at Sky

When the camera points at open sky (>30 degrees above horizontal):
- LiDAR: no return (beyond range)
- Neural: outputs maximum depth (meaningless)
- Scene classifier: detects SKY → zeros neural weight
- Geometric: not applicable (looking up)
- DEM: rejected by pitch guard (>30 degrees above horizontal)
- Result: no estimate displayed (correctly indicates no target)

Note: Looking at mountains (+5 to +25 degrees) is NOT "pointing at sky" — the DEM ray will intersect the terrain slope. The sky failure only applies when the camera pitch exceeds +30 degrees, where no terrain intersection is plausible.

### 4.5 Case Study: Mountain Over Rock Wall (Terrain Routing)

**Scenario:** Observer at 1000m elevation looking over a rock wall (~3m away) toward mountain cliffs at ~1600m. Camera pitch near level or slightly upward (+2 to +5 degrees).

**Without terrain routing (original architecture):**

| Source | Estimate | Weight | Notes |
|---|---|---|---|
| LiDAR | 2.5m | 0.98 | Sees the rock wall (sweet spot distance) |
| Neural | 42m | 0.35 | Saturated by inverse-depth compression |
| Geometric | 10m | 0.45 | Pitch-based, sees nearby ground |
| DEM | nil | — | **Blocked by pitch guard** (looking level/up) |
| **Fused** | **~15m** | **1.78** | Catastrophically wrong |

Even if DEM had been available, it would have been outvoted 3-to-1 in the weighted average.

**With terrain routing + pitch guard fix:**

| Source | Estimate | Weight | Notes |
|---|---|---|---|
| LiDAR | 2.5m | 0.98 | Sees rock wall (shown in inner bracket) |
| Neural | 42m | 0.35 | Foreground saturated |
| DEM | 1600m | 0.47 | Ray marches forward+up, hits mountain slope |
| **Terrain routed** | **1600m** | **0.40** | DEM returned directly, other sources bypassed |

The depth zone brackets show inner bracket at ~3m (amber, foreground) and outer bracket at ~1600m (cyan, terrain), giving the operator visual context for both distances.

### 4.4 Failure: Close Range Indoor

At 2m indoors:
- LiDAR: excellent (0.98 confidence)
- Neural: low weight (0.30 at 2m)
- Geometric: zero (< 5m)
- DEM: zero (< 20m)
- Result: LiDAR dominates as intended

---

## 5. Barometric Altimeter Integration

### 5.1 Motivation

DEM ray-casting accuracy is directly limited by altitude accuracy because the ray origin height determines where the descending ray intersects terrain. GPS altitude accuracy of 10–30m introduces proportional range errors:

```
range_error approximately altitude_error / sin(pitch)
```

At 5 degrees pitch: range_error = altitude_error / 0.087 = 11.5x altitude error.

With GPS (15m vertical accuracy): range_error up to 172m.
With barometer (3m vertical accuracy): range_error up to 34m.

This 5x improvement justifies barometric altimeter integration.

### 5.2 Implementation

iOS `CMAltimeter` provides two complementary altitude signals:

1. **Absolute altitude** (`startAbsoluteAltitudeUpdates`): GPS+barometer fusion with 1–5m accuracy. Available on iPhone with barometric sensor.

2. **Relative altitude** (`startRelativeAltitudeUpdates`): pressure-based with approximately 0.1m precision for altitude changes. Useful for detecting vertical motion but not absolute position.

The `LocationManager` maintains both barometric and GPS altitude, selecting the most accurate source via `updateBestAltitude()`. The selected vertical accuracy flows into `DEMRaycastEstimator.computeConfidence()` as the `altFactor` multiplier.

---

## 6. Operator Guidance Engine

### 6.1 Motivation

Military range-finding doctrine emphasizes operator technique as a primary accuracy factor. The AN/PSQ-23 STORM, Vectronix VECTOR, and SIG KILO rangefinders all include confidence/quality indicators to help operators judge reading reliability. We extend this concept by analyzing device motion to provide real-time coaching.

### 6.2 Stability Classification from IMU

The gyroscope provides angular velocity magnitude at 60 Hz. A 5-sample moving average classifies stability into 5 levels:

| Level | Angular Velocity | Stability % | Typical Scenario |
|---|---|---|---|
| Excellent | < 0.02 rad/s | 93–100% | Braced against solid object, tripod |
| Good | 0.02–0.05 rad/s | 83–93% | Steady two-hand hold |
| Adequate | 0.05–0.10 rad/s | 67–83% | Tracking slowly |
| Marginal | 0.10–0.30 rad/s | 0–67% | Walking, one-hand hold |
| Unstable | > 0.30 rad/s | 0% | Panning, shaking |

The stability percent (0–100%) drives a visual bar indicator in the HUD.

### 6.3 Respiratory Pause Detection

In military marksmanship, the natural respiratory pause (2–3 seconds at bottom of exhale) is the optimal moment for taking a reading. The detection algorithm identifies a motion→stillness transition:

1. Compute recent 3-sample average angular velocity
2. Compute earlier 5-sample average (3–8 samples ago)
3. If current < 0.02 rad/s AND earlier > 0.05 rad/s → respiratory pause detected
4. Display "CAPTURE WINDOW" hint with pulsing stability bar

This pattern distinguishes deliberate stabilization after breathing from continuous stillness.

### 6.4 Reading Lock

A range reading is "locked" when the displayed value has been stable within 5% relative change for 2+ seconds (minimum 5 readings in the window). This provides operator confirmation that the reading is reliable without requiring manual "lase" action.

### 6.5 Hint Priority System

14 guidance hints are ranked by priority (100 = highest). The display shows a maximum of 2 hints simultaneously, with a 2-second minimum duration to prevent flicker. Hints are classified into three severity levels (positive/caution/warning) mapped to green/amber/red display colors.

The hint system addresses: device stability, breathing technique, calibration status, GPS/compass quality, ambient light, and long-range technique (multiple readings recommendation at > 200m).

---

## 7. Performance Characteristics

### 7.1 Compute Budget

| Component | Rate | Typical Latency | Hardware |
|---|---|---|---|
| ARFrame delivery | 60 Hz | <1ms | ARKit |
| LiDAR sampling | 60 Hz | <0.1ms | Pointer read |
| Neural depth inference | 15 Hz | 20–40ms | Neural Engine |
| Object detection | 5 Hz | 30–50ms | Neural Engine |
| DEM ray-casting | 2 Hz | 5–15ms | CPU (async) |
| Geometric estimation | 60 Hz | <0.01ms | CPU (trivial) |
| Scene classification | 20 Hz | <0.1ms | CPU (sampling) |
| Kalman filter | 60 Hz | <0.01ms | CPU |
| Fusion + outlier | 60 Hz | <0.1ms | CPU |
| Operator guidance | 30 Hz | <0.1ms | CPU (IMU analysis) |

Total pipeline latency: 25–50ms (neural inference dominates). The system maintains 60 FPS UI with 15 FPS depth updates.

### 7.2 Memory

| Resource | Size |
|---|---|
| DepthAnythingV2SmallF16 | ~45MB (CoreML) |
| SniperScope_Detector | ~12MB (CoreML) |
| SRTM tile (each) | ~25MB (raw HGT) |
| SRTM tile cache (max 3) | ~75MB |
| EPQS elevation cache | <1MB |
| Calibration window (50 pairs) | <1KB |
| Kalman filter state | 64 bytes |
| Depth ring buffer (5 frames) | 40 bytes |

### 7.3 Network

- SRTM HGT tiles: loaded from local storage (no network if pre-installed)
- USGS EPQS: single HTTP GET per ~100m grid point. Typically <50 requests per session. Response size: <500 bytes each.

---

## 8. Test Methodology

### 8.1 Unit Test Coverage

312 tests across 24 test files (5 Tier 2/3 skipped without dataset) verify:

- **Confidence curves:** Each source curve tested at multiple distance points for expected shape, boundary conditions, and decay rates
- **Calibration:** Inverse and metric model detection, weighted least-squares fit accuracy, confidence computation, edge cases (single sample, zero neural depth)
- **Kalman filter:** Initialization, prediction accuracy, motion-adaptive noise, reset after long gaps, velocity tracking
- **Geometric estimator:** Distance computation at known angles, confidence tiers, slope penalty activation, out-of-range rejection
- **DEM ray-casting:** Confidence factor tiers (GPS, altitude, heading, distance), vertical accuracy integration, rate limiting
- **Scene classifier:** Synthetic depth map generation (uniform sky, gradient ground, discontinuous structure), rate limiting, classification correctness, pitch gating
- **Disagreement penalty:** Outlier suppression threshold, two-source disagreement, multi-source median computation
- **Inclination correction:** Cosine correction at various angles, small-angle bypass, formatting
- **Motion-aware smoothing:** Alpha adaptation by motion state, discontinuity detection, snap-through behavior
- **Barometric altimeter:** Default state, altitude source selection, vertical accuracy ranges, DEM confidence boost
- **Monte Carlo fusion:** Full-pipeline simulation with semantic selection across 10 distance bands, error validation, confidence distribution analysis
- **Terrain routing:** DEM-primary decision logic, pitch guard thresholds, ray direction math, uncertainty computation, depth zone bracket disagreement detection, edge cases (indoor, poor GPS, upward pitch)
- **Depth zone brackets:** DepthZoneOverlay struct, disagreement detection (>2× ratio), bracket activation, AppState integration
- **Semantic selection:** Priority chain ordering, source switch Kalman reset, background hypothesis source differs from primary, DEM priority in far-target mode, neural hard cap enforcement
- **Stadiametric ranging:** Pinhole formula at various ranges (10–3888m), zero-pixel edge case, target presets, focal length independence, pixel-error sensitivity analysis
- **Ballistics solver:** Holdover values validated against published .308/5.56/6.5CM tables at 200–1000yd, hold direction logic, monotonicity, cross-caliber comparisons, metric mode, edge cases
- **SRTM tile cache:** Tile parsing, elevation queries, bilinear interpolation, LRU cache eviction

### 8.2 Monte Carlo Fusion Validation

A dedicated Monte Carlo test harness validates the full fusion pipeline across 10 distance bands with two tiers:

- **CI tier (10,000 samples):** Runs in 0.25 seconds for continuous integration. The `FusionSimulator` includes the terrain routing early-return, matching the production `UnifiedDepthField.sampleAtCrosshair()` pipeline. Validates per-band error thresholds and zero catastrophic errors.
- **Deep analysis tier (1,000,000 samples):** Runs in ~22 seconds. Uses histogram-based streaming statistics (O(1) memory per category) for percentile distributions, source combination analysis, slope/GPS impact, and confidence calibration per band.

**1M Sample Results (4.89% global mean error, 0 catastrophic):**

| Band | Range | Mean | P50 | P90 | P95 | P99 |
|---|---|---|---|---|---|---|
| 1 | 0.3–3m | 0.4% | 0.2% | 0.6% | 1.0% | 1.0% |
| 2 | 3–8m | 1.0% | 1.0% | 2.2% | 2.6% | 3.8% |
| 3 | 8–15m | 2.7% | 1.8% | 6.2% | 7.8% | 10.2% |
| 4 | 15–30m | 6.2% | 5.0% | 13.8% | 16.6% | 21.0% |
| 5 | 30–50m | 7.6% | 4.6% | 18.6% | 24.2% | 36.2% |
| 6 | 50–100m | 6.2% | 3.4% | 13.0% | 24.2% | 52.6% |
| 7 | 100–200m | 5.7% | 1.8% | 7.8% | 23.8% | 84.2% |
| 8 | 200–500m | 5.8% | 1.0% | 5.4% | 37.4% | 91.0% |
| 9 | 500–1000m | 6.2% | 0.6% | 12.6% | 41.4% | 92.6% |
| 10 | 1000–2000m | 7.0% | 0.2% | 31.4% | 39.8% | 53.0% |

**Key findings from 1M analysis:**
1. **GPS quality is the dominant accuracy predictor**: Poor GPS (>10m) causes 12–31× more error than good GPS at long range. DEM confidence factors were steepened (GPS: 0.70→0.65 for 10–15m accuracy, heading: 0.60→0.55 for 10–15°).
2. **DEM+Object corroboration is critical**: 3–4× lower error than DEM-only. A 15% single-source DEM confidence penalty was added at >100m.
3. **Gentle slopes (3–8°) produce more error than steep slopes at 15–30m**: Geometric ranging has a confidence sweet spot that can produce overconfident wrong readings when it slightly disagrees with neural depth.

**Lesson learned:** The original DEM confidence curve dropped too aggressively at 600–2000m (0.88→0.50), and the maxExpected normalizer at 500m+ was 1.35 (assuming DEM+Object always fire together). This caused displayed confidence to read 30–40% on readings that were actually within 5% of ground truth. Flattening the DEM curve (0.88→0.65) and splitting 500m+ into two tiers (1.10 and 0.95) fixed the confidence-accuracy mismatch without affecting the fused distance values.

### 8.3 Comprehensive Distance Sweep Testing

A dedicated sweep test suite (`RangefinderSweepTests.swift`) validates the full ranging pipeline across 100–1500 yards with five complementary test types based on industry-standard rangefinder testing methodologies (GolfLink, Vovex Golf, KITTI, Mossy Oak/Bushnell):

**Test 1: General Ranging Sweep (100–1500 yds)**
- 57 distance bands in 25-yard steps, 50 frames × 3 operator seeds per band
- Distance-scaled tolerances: 8% (<200 yds), 12% (200–400), 18% (400–800), 25% (800–1200), 35% (1200–1500)
- Result: 57/57 bands pass, global mean 5.0%, P90 10.5%

**Test 2: Golf Pin Stadiametric Sweep (40–300 yds)**
- Tests USGA 2.13m flagstick at 1×, 5×, and 8× zoom with realistic bracket noise model
- Golf tolerances: 3% (<100 yds), 5% (100–150), 8% (150–200), 12% (200–250), 15% (250–300)
- Results: 5× zoom 27/27 pass (all club-selection grade), 8× zoom 27/27 pass, 1× zoom 21/27 pass (expected — pin too small beyond 150 yds at 1×)

**Test 3: Multi-Operator Repeatability**
- 5 operators (RNG seeds) × 6 key distances × 100 frames each
- Acceptance: coefficient of variation (CV) < 15% across operators
- All distances pass

**Test 4: Environmental Condition Matrix**
- 6 distances × 4 GPS qualities × 3 slopes × 3 calibration ages × 2 object configs = 432 combos × 20 frames
- Validates graceful degradation under extreme conditions (GPS 25m + steep slope + stale cal)
- Catastrophic errors (>80%) allowed up to 15% of combos (extreme conditions expected to degrade)

**Test 5: Source Handoff Stress Test**
- Walks 5m → 1372m in 10m steps (30 frames each), verifying no dead zones and no 3× jumps
- Validates source transition ordering: LiDAR → Neural → Geometric → DEM/Object

### 8.4 Synthetic Depth Maps

Scene classifier tests use programmatically generated CVPixelBuffer depth maps:

```swift
func createDepthMap(fill: (Int, Int, Int, Int) -> Float) -> CVPixelBuffer {
    // 64x64 Float32 depth map with custom fill pattern
    CVPixelBufferCreate(kCFAllocatorDefault, 64, 64,
                        kCVPixelFormatType_DepthFloat32, nil, &buffer)
    // Fill with closure-provided values
}
```

Test patterns:
- **Sky:** uniform 100.0m depth across all pixels
- **Ground:** linear gradient from 5m (bottom) to 55m (top)
- **Structure:** left half 10m, right half 80m (sharp discontinuity)
- **Mixed:** pseudo-random pattern `(x*7 + y*13) % 17`

### 8.5 Ground Truth Dataset Validation

A 10,000-sample ground truth dataset validates the fusion pipeline against real-world depth measurements with laser-scanner reference data. This extends Monte Carlo validation (synthetic noise models) with empirically measured depth distributions.

**Dataset composition:**
- **ARKitScenes (Apple):** iPad Pro LiDAR + Faro Focus S70 laser scanner ground truth (±1mm). 365K+ frames across 1,661 scenes.
- **DIODE (MIT-licensed):** Indoor/outdoor scenes with laser scanner ground truth. 25K+ samples spanning 0.5–350m.

**Manifest structure:** 10,000 samples stratified across 6 distance bands (close 0.5–3m, near_mid 3–8m, mid 8–15m, far_mid 15–50m, far 50–150m, long 150–350m) with per-sample ground truth center distance, optional LiDAR readings, depth percentiles (P25/P75), scene type, camera intrinsics, and dataset provenance.

**Three-tier test architecture:**

| Tier | Data Required | Tests | Runtime |
|---|---|---|---|
| Tier 1 | Manifest only (3.5MB, bundled) | 7 tests: integrity, distribution, confidence, source selection, fusion accuracy, statistical accuracy (50K evaluations), LiDAR accuracy | ~0.5s (CI) |
| Tier 2 | Real depth maps (~500MB) | 3 tests: depth map sampling, bimodal detection, noise characterization | ~30s |
| Tier 3 | Full images (~5GB) + device | 2 tests: neural inference, full pipeline E2E | On-device only |

Tier 2/3 tests use `XCTSkip` when `RANGEFINDER_DATASET_PATH` is not set, ensuring CI runs complete without large dataset dependencies.

**Per-band accuracy thresholds:**

| Band | Range | Max AbsRel | Max P90 | Max Catastrophic |
|---|---|---|---|---|
| close | 0.5–3m | 5% | 8% | 0% |
| near_mid | 3–8m | 8% | 12% | 0% |
| mid | 8–15m | 12% | 20% | 0% |
| far_mid | 15–50m | 15% | 25% | 1% |
| far | 50–150m | 20% | 30% | 2% |
| long | 150–350m | 25% | 40% | 5% |

**Statistical validation:** The `testPerBandStatisticalAccuracy` test runs 5 Monte Carlo samples per ground truth entry (50,000 total evaluations) with randomly varied GPS accuracy, heading accuracy, and terrain slope, validating that the fusion pipeline produces stable results across condition variations.

**Relationship to Monte Carlo tests:** Monte Carlo tests (Section 8.2) validate the fusion algorithm with synthetic noise models across 0.3–2000m. Ground truth tests validate the same algorithm against empirically measured depth distributions from real sensors, providing complementary coverage. Monte Carlo covers extreme ranges (>350m) that no public depth dataset reaches; ground truth covers the realistic noise characteristics that synthetic models approximate.

---

## 9. Known Limitations

1. **Neural calibration extrapolation:** Beyond 8m, the inverse-depth calibration is extrapolating. Accuracy degrades continuously. The 200m cap in `ContinuousCalibrator.calibrate()` prevents runaway extrapolation but limits neural contribution at extreme range.

2. **Geometric flat-ground assumption:** Even with slope penalty, geometric ranging is fundamentally wrong on non-flat terrain. The penalty reduces its fusion weight but cannot correct the estimate itself.

3. **DEM resolution (30m):** SRTM 1-arc-second data cannot resolve features smaller than 30m (buildings, ditches, berms). Targets on or behind such features may receive incorrect DEM estimates.

4. **Magnetometer heading accuracy:** iPhone magnetometer accuracy varies from 2–15 degrees depending on magnetic interference. At 500m range, 10 degree heading error creates 87m lateral displacement. Indoor or vehicle environments degrade heading severely.

5. **Object detection coverage:** Size-based ranging only works when recognizable objects are in the field of view and detected with adequate confidence. The KnownObjectDatabase covers people, vehicles, signs, and common animals but is not exhaustive.

6. **Single-point sampling:** The system samples depth at screen center (crosshair). Off-center targets require the user to point the device directly at the target. A future multi-point sampling mode could support area ranging.

7. **Barometric altimeter availability:** `CMAltimeter.absoluteAltitudeData` requires iOS 15+ and hardware barometer. The absolute altitude API may not be available in all regions or conditions.

---

## 10. Future Work

### 10.1 Data-Driven Refinements (Informed by 1M Monte Carlo)

1. **Geometric confidence at gentle slopes:** The 15–30m band shows the highest mean error (6.2%) due to geometric overconfidence when pitch is 2–5°. A steeper pitch-based penalty for near-flat angles would reduce this band's error without affecting other ranges.

2. **GPS-aware displayed confidence:** A separate post-fusion confidence modifier that further reduces displayed confidence when GPS accuracy > 10m. The current fusion weight penalty (0.65/0.35) is intentionally moderate to avoid starving DEM of weight; a display-only penalty can be more aggressive.

3. **Field validation dataset expansion:** The 10K-sample ground truth manifest (Section 8.5) provides offline validation against laser-scanner reference measurements from ARKitScenes and DIODE. The next step is on-device captures at known distances using a reference laser rangefinder across varied terrain, validating real ARKit + CoreML inference accuracy against the Monte Carlo noise model predictions.

### 10.2 Feature Development

4. **Deliberate "LASE" capture button:** A manual capture mode that samples multiple frames during the stability window and averages readings, mimicking the deliberate trigger action of laser rangefinders.

5. **Dynamic BDC reticle marks:** Bullet Drop Compensator marks that adjust position based on selected caliber and environmental conditions, similar to the Burris Eliminator scope.

6. **Operator guidance unit tests:** Dedicated test coverage for stability classification, respiratory pause detection, and reading lock logic with synthetic IMU data streams.

7. **Multi-target tracking:** Maintain range estimates for multiple crosshair positions simultaneously, enabling rapid target switching without re-acquisition delay.

8. **Wind estimation integration:** Incorporate wind speed/direction sensors or manual input for the ballistic solver's lateral drift calculation.

### 10.3 Research Directions

9. **Stereo fusion from dual cameras:** Use disparity between wide and ultrawide cameras for geometric depth estimation independent of IMU pitch.

10. **Machine learning confidence prediction:** Train a model to predict fusion confidence from input features (source count, agreement, distance, motion state) rather than using hand-tuned curves.

11. **Multi-frame neural depth:** Aggregate neural depth across multiple frames with known camera motion for improved long-range estimates.

12. **Collaborative calibration:** Share calibration parameters between devices to bootstrap neural calibration without LiDAR overlap.

13. **SRTM tile download manager:** In-app tile browsing and download for offline DEM coverage.

14. **Adaptive confidence curves:** Use accumulated field data to auto-tune confidence curve parameters per-device and per-environment, replacing static piecewise curves with learned profiles.

---

## 11. Codebase Summary

| Metric | Value |
|---|---|
| Source files | 52 Swift files |
| Source lines | ~11,700 |
| Test files | 24 Swift files |
| Test lines | ~8,000 |
| Total lines | ~19,700 |
| Unit tests | 312 (5 Tier 2/3 skipped without dataset) |
| Ground truth samples | 10,000 (ARKitScenes + DIODE, laser-scanner reference) |
| Depth sources | 6 (LiDAR, neural, geometric, DEM, object, stadiametric) |
| ML models | 3 (depth, depth secondary, object detection) |
| Frameworks | 14 Apple frameworks + MapKit |
| Minimum iOS | 18.0 |
| Swift version | 6.0 (strict concurrency) |
| Build system | XcodeGen (project.yml) |
| Target devices | iPhone with LiDAR (12 Pro+) |
| Reticle styles | 3 (mil-dot, bracket, rangefinder) |
| Guidance hints | 14 types across 3 severity levels |
| Selection architecture | Semantic source selection (priority state machine) |
| Kalman filters | 2 (foreground + background hypothesis) |

---

## 12. Lessons Learned

### 12.1 Displayed Confidence vs. Actual Accuracy

The most impactful bug was the mismatch between displayed confidence and actual accuracy at long range. Monte Carlo testing showed 5–7% error at 500–2000m, but the HUD displayed 30–48% confidence. The root cause was a conservative `maxExpected` normalizer that assumed DEM + object detection always fire together. In practice, object detection is opportunistic — when only DEM fires (common at extreme range), the normalizer was too high, suppressing the displayed confidence.

**Fix:** Split the 500m+ normalizer into two tiers (500–1000m: 1.10, 1000m+: 0.95) so DEM-alone produces 54–60% confidence. This better represents the actual sub-10% accuracy.

**Principle:** Confidence display should be calibrated to actual error rates, not to theoretical maximum source availability. A conservative normalizer that assumes "all sources should fire" will under-report confidence whenever the full source complement is unavailable.

### 12.2 DEM Long-Range Decay Too Aggressive

The DEM confidence curve originally dropped from 0.88 → 0.50 over 600–2000m. Monte Carlo testing with realistic noise models proved DEM ray-march readings were < 1% error even at 1500m thanks to step-averaging through terrain. The fix was flattening the curve to 0.88 → 0.65.

**Principle:** Validate confidence curves against measured accuracy, not intuition about error growth. The ray-march algorithm's inherent averaging over terrain steps provides noise suppression that the original hand-tuned curve did not account for.

### 12.3 Fusion Weight vs. Displayed Confidence

The 1M Monte Carlo analysis revealed that GPS quality causes 12–31× more error at long range. The initial fix — aggressive GPS/heading penalty factors — backfired: global mean error jumped from 4.63% to 6.13% because steeper penalties reduced DEM *fusion weight*, not just displayed confidence. With less weight, DEM contributed less to the fused estimate, and the remaining sources (neural extrapolation) were worse.

**Fix:** Moderate fusion weight penalties (GPS: 0.85/0.65/0.35; heading: 0.80/0.55/0.35) plus a separate post-fusion DEM-only confidence penalty (×0.85) for display accuracy.

**Principle:** Fusion weight and displayed confidence serve different purposes. Fusion weight controls *how much* a source contributes to the estimate — penalizing this too aggressively removes the best available source. Displayed confidence controls *what the user sees* — this can be penalized independently to reflect input quality without degrading the estimate itself.

### 12.4 DEM + Object Corroboration Effect

The 1M analysis showed that DEM+Object corroboration produces 3–4× lower error than DEM-only at 100m+. When both sources agree, the multi-source agreement bonus (+15%) is justified; when DEM stands alone, the 15% single-source penalty reduces displayed confidence proportionally. This finding motivated splitting the `maxExpected` normalizer and adding the DEM-only penalty rather than uniformly boosting or suppressing DEM confidence.

### 12.5 Ballistics Solver: Three Compounding Bugs

The original ballistics solver had three bugs that compounded to produce holdover values 2–3× too high:

1. **Linear drag model with wrong constant:** The simplified drag formula `avgVelocity = MV × (1 - (yards/1000) × (1-BC))` with `K=166,000` barely decelerated the bullet (2650→2548 fps at 1000yd, reality: 2650→~1550 fps). The constant was 12× too large, making drag negligible.

2. **Sight-height geometry error:** The holdover formula `drop(D) - drop(Z) × (D/Z)` subtracted sight height inside the drop calculation, then linearly scaled the modified value. The correct formula requires: `drop(D) + h - (drop(Z) + h) × (D/Z)`, where `h` is sight height. The missing `h×(1 - D/Z)` term added 3–27" of phantom drop.

3. **Inverted hold direction:** `holdHigh = holdoverValue < 0` — the opposite of correct. At 500yd the display said "hold low" when the bullet was actually below the sight line.

**Fix:** Full G1 drag table (65 Mach/Cd data points from the Ingalls/Mayevski standard) with step-integrated point-mass solver (1-yard increments), proper sight-line geometry, and corrected direction logic. The new solver produces values matching published ballistic tables within ±20% (G1 vs G7 form factor difference for boat-tail bullets).

**Principle:** Ballistic solvers must be validated against known reference data at multiple distances. A simplified model that appears correct at 200yd can be wildly wrong at 1000yd because errors in velocity estimation grow nonlinearly with time of flight.

### 12.6 DEM Pitch Guard: Conservative Guards Block Valid Scenarios

The original DEM ray-cast estimator had a pitch guard that only allowed downward-looking rays (`pitch < -0.17 degrees`). This was a conservative safety check to prevent rays that "never intersect terrain." In practice, it blocked the entire mountain/cliff scenario — when looking level or upward toward mountains, DEM returned nil, terrain routing never fired, and the fusion reported the foreground rock wall distance.

The ray-march math already handled upward rays correctly. When pitch is positive, the Up component of the direction vector is positive, and the ray marches forward and upward until it intersects rising terrain (mountain slope). The guard was relaxed to +30 degrees above horizontal.

**Principle:** Conservative safety guards should be validated against real-world use cases, not just theoretical failure modes. A guard that "prevents bad results" by blocking valid scenarios is worse than no guard at all. The natural termination condition (ray exceeds maxRayDistance without hitting terrain → returns nil) provides adequate safety without blocking upward-looking mountain scenarios.

### 12.7 Weighted Average Cannot Overcome Fundamental Source Disagreement

The original fusion architecture treated all 5 depth sources as equal votes in a weighted average. When 3–4 sources see a foreground obstacle (rock wall at 2m) and only DEM sees the terrain behind it (1600m), no amount of confidence curve tuning, DEM-dominance rules, or outlier suppression can make the weighted average produce 1600m. The fundamental insight: for terrain targets without discrete objects, the DEM ray-cast IS the answer, not one voice in a committee.

Terrain routing implements this as an early-return that short-circuits the entire fusion pipeline. The other sources still appear in the depth zone bracket overlay for operator context (inner bracket = foreground, outer bracket = terrain) but do not participate in the ranging computation.

**Principle:** When one source has a fundamentally different measurement basis (DEM traces actual terrain topology vs. neural/LiDAR point-sampling at a pixel), the correct architecture is source selection, not source averaging. Averaging is appropriate when sources measure the same thing with different noise characteristics; it fails when they measure different things entirely.

### 12.8 Operator Technique Matters More Than Algorithm Tuning

Research into military rangefinding doctrine (FM 23-10, Vectronix VECTOR manual, SIG KILO procedures) consistently emphasizes that operator technique — breathing, bracing, multiple readings — is the primary accuracy differentiator beyond the instrument's inherent capability. Adding real-time coaching based on IMU analysis addresses the human factor that no amount of algorithm optimization can fix.

### 12.9 Source Selection vs. Source Averaging

The most significant architectural lesson was that weighted-average fusion fails when sources measure fundamentally different physical quantities. Averaging a 2m rock wall (LiDAR/neural foreground) with 1600m terrain (DEM) produces ~25m — useless to the operator. The replacement semantic source selection architecture picks ONE authoritative source per frame based on a priority chain, which correctly returns 1600m when DEM is selected and 2m when LiDAR is selected. The operator sees both readings (primary + background hypothesis) and can make an informed decision.

**Principle:** Averaging is appropriate when sources measure the same thing with different noise characteristics. It fails when sources measure different things entirely (foreground vs. terrain). The correct architecture for heterogeneous measurements is selection, not fusion.

### 12.10 Neural Hard Cap Extension: 50m → 150m

The original 50m hard cap was effective but too aggressive — it excluded neural from the 50–150m range where it can still provide useful (if low-confidence) signal as a secondary contributor alongside DEM. The pipeline overhaul extended the cap to 150m with a gradual confidence decay (0.35 at 50m → 0.08 at 150m), while simultaneously removing the calibrator's compression cap that had been artificially squashing distances beyond ~80m. The combination allows neural to contribute useful medium-range signal without dominating fusion at ranges where its calibration is unreliable.

**Principle:** Hard caps should be set at the boundary where a source becomes genuinely harmful, not merely imperfect. Between 50–150m, neural with 0.15 confidence provides useful corroboration of DEM without overriding it.

### 12.11 Pipeline Overhaul: Seven Interacting Fixes

A comprehensive pipeline overhaul addressed seven interacting issues that collectively prevented accurate ranging beyond 50m:

1. **Neural hard cap 50→150m** — extended useful neural range
2. **Calibrator compression removed** — eliminated artificial distance squashing
3. **Confidence curves extended** — LiDAR 10→12m, neural gradual decay to 150m
4. **LiDAR max range 10→12m** — captures more calibration pairs
5. **Geometric slope penalty relaxed** — threshold 3→5°, avoiding false penalties at gentle angles
6. **Kalman distance factor capped at 25** — keeps filter responsive at long range
7. **Semantic switch detection before outlier rejection** — prevents legitimate source transitions from being rejected as outliers

**Principle:** Depth pipeline issues compound nonlinearly. The compression cap at 80m and the Kalman distance factor of 100 individually caused moderate problems, but together they made long-range readings both wrong (compressed) and frozen (unresponsive filter). Fixing either alone improved results; fixing both together produced dramatically better accuracy across the full 100–1500 yard range.

### 12.12 Dual Kalman Filters Enable Multi-Hypothesis Display

A single Kalman filter tracking the "fused" estimate cannot simultaneously represent foreground and background depths. When the semantic selection switches from neural (45m foreground) to DEM (1600m terrain), the single filter slowly tracks toward the new value, displaying incorrect intermediate readings for several frames. Dual independent filters — one for the primary selection, one for the background hypothesis — solve this by resetting the foreground filter on source switches and maintaining the background estimate independently.
