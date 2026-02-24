# Rangefinder — Instructions

## Overview

Rangefinder is an iOS application that turns an iPhone into a multi-source digital rangefinder. It uses **semantic source selection** to choose the best depth source per frame from six available sources — LiDAR, neural depth estimation, geometric ground-plane ranging, DEM terrain ray-casting, object-size ranging, and user-directed stadiametric bracket ranging — with multi-hypothesis tracking from 0 to 2000 meters.

The interface presents a configurable reticle overlay (mil-dot, crosshair, or rangefinder) on the live camera feed with real-time range readout, confidence indicators, semantic source decision label, background hypothesis chip, source blend visualization, stadiametric bracket overlay, DEM map picture-in-picture, compass bearing, operator guidance coaching, inclination correction, and optional ballistic holdover computation.

---

## Requirements

### Hardware
- **iPhone with LiDAR** — iPhone 12 Pro or later (Pro/Pro Max models)
- **ARKit support** — A12 Bionic or later
- LiDAR is used for close-range ground truth (0.3–5m) and continuous neural model calibration

### Software
- **iOS 18.0** or later
- **Xcode 16.0** or later
- **Swift 6.0** (strict concurrency mode enabled)
- **XcodeGen 2.38+** — project file generation from `project.yml`

### Permissions
The app requests:
| Permission | Purpose | When |
|---|---|---|
| Camera | Live viewfinder + LiDAR depth | On launch |
| Motion & Fitness | Device pitch/heading via IMU | On launch |
| Location (When In Use) | GPS position for DEM terrain ranging | On launch |

---

## Build & Run

### 1. Install XcodeGen

```bash
brew install xcodegen
```

### 2. Generate Xcode Project

```bash
cd Rangefinder
xcodegen generate
```

This reads `project.yml` and produces `Rangefinder.xcodeproj`.

### 3. Open in Xcode

```bash
open Rangefinder.xcodeproj
```

### 4. Configure Signing

- Select the `Rangefinder` target
- Under **Signing & Capabilities**, set your Development Team
- The bundle identifier is `com.mildotrangefinder.app`

### 5. Build & Run

- Select a physical device (LiDAR requires real hardware)
- Build: `Cmd+B`
- Run: `Cmd+R`

### Command-Line Build

```bash
xcodebuild build \
  -project Rangefinder.xcodeproj \
  -scheme Rangefinder \
  -destination 'generic/platform=iOS'
```

### Run Tests

```bash
xcodebuild test \
  -project Rangefinder.xcodeproj \
  -scheme Rangefinder \
  -destination 'platform=iOS Simulator,name=iPhone 17 Pro'
```

There are 269 unit tests covering all depth sources, semantic source selection, confidence curves, dual Kalman filtering, calibration, scene classification, geometric estimation, DEM ray-casting, stadiametric ranging, terrain routing, depth zone brackets, and Monte Carlo validation.

---

## Project Structure

```
Rangefinder/
├── App/                    App lifecycle and state management
│   ├── AppState.swift          Central @Observable state + Combine bindings
│   └── RangefinderApp.swift  SwiftUI entry point
├── Camera/                 ARKit camera capture pipeline
│   ├── CameraManager.swift     ARSession management + frame delivery
│   ├── CameraIntrinsics.swift  Focal length extraction for pinhole model
│   ├── CameraPreviewView.swift UIViewRepresentable for AR preview
│   └── ZoomController.swift    Multi-lens zoom (0.5x–25x)
├── Core/                   Configuration and utilities
│   ├── Configuration.swift     All numeric constants in one place
│   ├── Errors.swift            Error types
│   ├── Logger.swift            os.Logger categories (12 subsystems)
│   └── PerformanceMonitor.swift FPS and thermal tracking
├── Depth/                  Multi-source depth estimation
│   ├── UnifiedDepthField.swift     Semantic source selection state machine
│   ├── MetricDepthEstimator.swift  CoreML DepthAnythingV2 wrapper
│   ├── ContinuousCalibrator.swift  LiDAR→neural calibration (inverse depth)
│   ├── GeometricRangeEstimator.swift Ground-plane D = h/tan(θ)
│   ├── SceneClassifier.swift       Depth-map scene analysis (sky/ground/structure)
│   ├── LiDARDepthProvider.swift    ARFrame depth map extraction
│   ├── DepthSourceConfidence.swift Distance-dependent confidence curves
│   └── DepthTypes.swift            Shared depth types and enums
├── Extensions/             Swift extensions
│   ├── CVPixelBuffer+Extensions.swift  Pixel buffer helpers
│   └── Measurement+Extensions.swift    Unit formatting
├── Models/                 CoreML models (bundled)
│   ├── DepthAnythingV2SmallF16.mlpackage  Neural depth (primary)
│   ├── DepthPro_Lite.mlpackage            Neural depth (secondary)
│   └── SniperScope_Detector.mlpackage     Object detection
├── ObjectDetection/        Object recognition + size-based ranging
│   ├── ObjectDetector.swift        YOLO + Vision framework detector
│   ├── ObjectRangeEstimator.swift  Pinhole model ranging
│   └── KnownObjectDatabase.swift   ANSUR/EPA/MUTCD reference sizes
├── Prediction/             Temporal smoothing
│   ├── DepthKalmanFilter.swift     1D Kalman [depth, velocity]
│   └── IMUDepthPredictor.swift     Motion-based depth prediction
├── Ranging/                Pipeline orchestration + ballistics + operator coaching
│   ├── RangingEngine.swift         9-step per-frame pipeline
│   ├── RangingTypes.swift          DepthSource enum, RangeOutput
│   ├── InclinationCorrector.swift  Cosine correction for angled shots
│   ├── MotionAwareSmoother.swift   Adaptive smoothing by motion state
│   ├── BallisticsSolver.swift      G1 drag model + holdover
│   └── OperatorGuidanceEngine.swift IMU stability, breathing, coaching hints
├── Reticle/                Configurable reticle overlay (3 styles)
│   ├── FFPReticleView.swift        First focal plane reticle rendering (mil-dot/crosshair/rangefinder)
│   ├── ReticleConfiguration.swift  Style/color/appearance settings + ReticleStyle enum
│   └── ReticleGeometry.swift       Mil-to-pixel conversion
├── Sensors/                Hardware sensor interfaces
│   ├── InclinationManager.swift    CMMotionManager (pitch/heading)
│   └── LocationManager.swift       CLLocation + CMAltimeter (GPS/baro)
├── Terrain/                Elevation data + ray-casting
│   ├── SRTMTileCache.swift         HGT tile parser + USGS EPQS fallback
│   └── DEMRaycastEstimator.swift   Ray-terrain intersection algorithm
└── UI/                     SwiftUI views
    ├── RangefinderView.swift              Main camera + HUD view + compass bearing
    ├── HUDOverlayView.swift               Source blend bar + legend
    ├── BackgroundRangeChip.swift          Background hypothesis "BG" chip
    ├── StadiametricBracketOverlay.swift   Draggable bracket overlay for manual ranging
    ├── MapPiPView.swift                   DEM hit point satellite map overlay
    ├── RangeDisplayView.swift             Primary range readout
    ├── ConfidenceBadge.swift              Color-coded confidence indicator
    ├── HoldoverIndicator.swift            Ballistic holdover display
    ├── OperatorGuidanceView.swift         Stability bar + coaching hint chips
    ├── SettingsView.swift                 Config panel + stadiametric presets
    ├── TutorialView.swift                 7-page tactical onboarding tutorial
    └── Theme.swift                        MIL-STD-3009 color palette
```

---

## Using the App

### Main Screen

On launch, the app displays a live camera feed with a configurable reticle centered on screen. The primary range readout appears at the top center.

**Range Display:**
- Large number: estimated range in yards or meters
- Confidence dot: color-coded (green/amber/red)
- Uncertainty: inline ± value when significant
- Source blend bar: visual breakdown of source weights (bottom zone)
- **Depth zone brackets**: inner bracket = crosshair/fusion depth, outer bracket = DEM terrain depth. Brackets change color (amber/cyan) when disagreement is detected (>2× ratio between crosshair and DEM readings)

**HUD Elements:**

*Top Bar — Row 1 (core data chips):*
- **MAG**: current magnification (0.5x–25x)
- **TGT**: target priority mode (NEAR/FAR) — tappable toggle, amber indicator when bimodal
- **Ballistics chip**: caliber + zero distance (when enabled, tappable for quick selection)
- **HDG**: compass bearing + cardinal direction (e.g., 045°NE) — visible when GPS is active
- **ELEV**: pitch angle with color coding by severity

*Top Bar — Row 2 (mode toggles):*
- **STADIA**: toggle stadiametric bracket overlay (amber when active)
- **MAP**: toggle DEM map picture-in-picture (amber when active, requires GPS)

*Bottom Zone:*
- **Semantic decision label**: shows which source won (e.g., "DEM", "LIDAR", "NEURAL")
- **BG chip**: background hypothesis range + source (e.g., "BG DEM 600 YDS")
- **Source blend bar**: visual breakdown of active source weights + legend
- **Holdover**: ballistic correction in mils (when enabled)

**Operator Guidance (below range readout):**
- **Stability bar**: horizontal fill bar (red→amber→green) showing device hold quality, pulses during capture window
- **Coaching hints**: up to 2 simultaneous terse advisory chips (e.g., "HOLD STEADY", "CAPTURE WINDOW", "CAL AGING — WALK CLOSE")
- Hints are priority-ranked and auto-dismiss when conditions clear

### Zoom

- **Pinch gesture**: continuous zoom from 0.5x to 25x
- **Snap points**: tap lens indicators to snap to 0.5x, 1x, 5x hardware lenses
- Digital zoom engages beyond 8x optical

### Settings

Tap the menu icon (☰) to open settings:

- **Display Unit**: yards or meters
- **Camera Height**: adjustable for prone (0.3m), standing (1.5m), tripod, or vehicle mounting
- **Reticle Style**: MIL-DOT (NATO standard with dots/hashes), CROSSHAIR (clean, maximum clarity), RANGEFINDER (duplex + Vectronix-style ranging brackets)
- **Reticle Color**: phosphor green, red, amber, purple (NVG/DAY/RED presets)
- **Reticle Options**: line width, outline toggle; mil-dot style adds filled dots, hash marks, mil labels toggles
- **Ballistics**: enable/disable, caliber selection (.308 Win, 5.56 NATO, 6.5 CM, .300 WM, .338 NM, .338 LM), zero distance
- **Stadiametric Ranging**: target height slider (0.3–12.0m), presets (PERSON, VEHICLE, DEER, DOOR) with active-state highlighting
- **Terrain Ranging**: GPS status, altitude, DEM status, terrain data tile management
- **System Info**: model status, GPS quality, barometer status, detection capabilities

### Depth Sources

The app uses **semantic source selection** — a priority-based state machine that picks ONE authoritative source per frame (not a weighted average):

| Priority | Source | Range | When Selected |
|---|---|---|---|
| 1 | **Stadiametric** | Any | User activates STADIA mode and brackets a target |
| 2 | **LiDAR** | 0–8m | Close range, high confidence |
| 3 | **Object Detection** | 20–1000m | Known-size object detected at crosshair |
| 4 | **DEM Ray-Cast** | 20–2000m | Terrain target, no object detected |
| 5 | **Neural Depth** | 2–50m | Calibrated, hard-capped at 50m |
| 6 | **Geometric** | 5–500m | Ground-plane fallback (D = h/tan(pitch)) |

**Why not weighted average?** Averaging sources that measure fundamentally different things fails catastrophically. When LiDAR sees a rock wall at 2m and DEM sees terrain at 1600m, no weighted average can produce the correct answer. Semantic selection picks the right source for the scenario.

**Background Hypothesis:** A second source provides an alternate reading (e.g., "BG DEM 600 YDS" when neural is primary). This gives the operator context about both foreground and terrain depths.

**Neural Hard Cap (50m):** Neural depth estimation is excluded beyond 50m because inverse-depth noise amplification makes estimates unreliable at longer ranges.

### Stadiametric Ranging

Activate STADIA mode via the top bar toggle. A bracket overlay appears on screen:

1. Select target type in Settings (PERSON 1.8m, VEHICLE 1.5m, DEER 1.0m, etc.)
2. Drag the top and bottom brackets to align with the target's top and bottom
3. Range is computed instantly: `R = (targetHeight × focalLength) / pixelSpan`

Stadiametric input has the highest priority — it overrides all sensor-based sources.

### Map Picture-in-Picture

Activate MAP mode via the top bar toggle (requires GPS). A satellite map overlay shows:
- Your position (blue dot)
- DEM ray-cast hit point (red pin)
- Connecting line (amber)

This visually confirms the DEM ray is hitting the expected terrain feature.

### Scene Classification

The scene classifier analyzes the neural depth map to detect:
- **SKY**: uniform maximum depth + upward pitch — zeros neural confidence
- **GROUND**: monotonic depth gradient + downward pitch — boosts geometric confidence
- **STRUCTURE**: sharp depth discontinuity — suppresses geometric, favors neural + DEM

### DEM Terrain Ranging

When GPS is available, the app casts rays from the device position through SRTM elevation data to find terrain intersections. This is the primary method for sloped terrain where geometric ground-plane ranging fails. The ray-cast supports both downward-looking and upward-looking rays (up to +30 degrees above horizontal), enabling ranging to mountains and cliffs above the observer.

**Data sources:**
- Local SRTM HGT tiles (bundled or placed in Documents/SRTM/)
- USGS EPQS API fallback (US coverage, no API key required)

To add SRTM tiles for your area:
1. Download 1-arc-second HGT files from [USGS EarthExplorer](https://earthexplorer.usgs.gov/)
2. Place them in the app's Documents/SRTM/ directory (e.g., `N37W122.hgt`)
3. Each tile covers 1 degree and is approximately 25MB

### Barometric Altimeter

On devices with barometric sensors (iPhone 6 and later), the app uses CMAltimeter for altitude:
- **Absolute altitude**: fused GPS+barometer with 1–5m vertical accuracy
- **Relative altitude**: pressure-based with ~0.1m precision
- Automatically selects the most accurate altitude source
- Improves DEM ray-casting confidence (1–5m barometric vs 10–30m GPS-only)

The altitude source indicator shows GPS, BARO, or FUSED in the HUD.

### Operator Guidance

The operator guidance engine analyzes IMU data in real-time to coach the user on improving ranging accuracy:

**Stability Levels:**
| Level | Angular Velocity | Description |
|---|---|---|
| EXCELLENT | < 0.02 rad/s | Braced/tripod quality |
| GOOD | 0.02–0.05 rad/s | Steady handheld |
| ADEQUATE | 0.05–0.10 rad/s | Tracking, moderate stability |
| MARGINAL | 0.10–0.30 rad/s | Moving, usable |
| UNSTABLE | > 0.30 rad/s | Panning/shaking |

**Coaching Hints (14 types, 3 severity levels):**
- **Positive** (green): STABILIZED, READING LOCKED, CAPTURE WINDOW, CALIBRATING
- **Caution** (amber): HOLD STEADY, CAL AGING, MULTIPLE READINGS REC, GPS LOW ACCURACY, COMPASS INTERFERENCE
- **Warning** (red): EXCESSIVE MOTION, BRACE DEVICE, LOW LIGHT, GPS ACQUIRING, CAL NEEDED

**Breathing Detection:** Detects the natural respiratory pause (bottom-of-exhale stillness window) used in military marksmanship — displays "CAPTURE WINDOW" when optimal.

**Reading Lock:** When the displayed range has been stable within 5% for 2+ seconds, displays "READING LOCKED" as confirmation.

### Reticle Styles

Three configurable reticle styles, all rendered in First Focal Plane (FFP) — the reticle scales proportionally with zoom so angular measurements remain calibrated at any magnification:

| Style | Description | Best For |
|---|---|---|
| MIL-DOT | NATO standard with 1-mil dots + optional half-mil hashes + mil labels | Angular measurement, wind holds, range estimation by subtension |
| CROSSHAIR | Clean duplex crosshair, no marks | Maximum target clarity, digital-only ranging |
| RANGEFINDER | Duplex crosshair + Vectronix-style L-shaped corner brackets (2×2 mil square) | Quick angular size reference, target framing |

All reticle styles include always-on depth zone brackets that show the relationship between the crosshair reading and DEM terrain distance.

### Ballistics

When enabled, the ballistics solver computes holdover for the selected caliber using a step-integrated G1 point-mass trajectory model with full Mach-dependent drag table:
- G1 drag table (65 Mach/Cd data points) with 1-yard step integration
- Proper sight-line geometry for holdover computation
- Configurable zero distance (yards or meters, follows display unit)
- Multi-unit display: mils (primary, for reticle reference) + inches or cm (secondary)
- Direction indicator: UP (hold high) or DN (hold low)
- Inclination-corrected (cosine adjustment applied before ballistic computation)
- HUD format: `▲ 2.9 MIL UP · 52.0 IN`

---

## Troubleshooting

| Issue | Solution |
|---|---|
| "Camera access denied" | Settings → Rangefinder → Camera → Allow |
| Range shows "---" | Insufficient depth data — ensure camera has clear view |
| Low confidence at 50+ meters | Neural model calibrating — walk past objects at 1–5m first |
| DEM showing "NO GPS" | Ensure Location permission is granted and GPS has fix |
| No terrain data | Add SRTM HGT tiles for your area or ensure internet access for EPQS |
| Range jumps erratically | Hold device steady — watch the stability bar, brace against a solid surface |
| "CAL NEEDED" hint showing | Walk past objects at 1–5m range to feed LiDAR calibration pairs |
| "BRACE DEVICE" hint persists | Lean against wall/tree/vehicle, or use prone/kneeling position |
| Stability bar stays red | Reduce hand motion — use two-hand grip, exhale and pause before reading |
| Range reads foreground object, not terrain | Ensure GPS has fix — DEM terrain routing requires GPS. Watch for depth zone brackets: inner = foreground, outer = terrain |
| Build fails after adding files | Run `xcodegen generate` to regenerate the Xcode project |

---

## Test Coverage

269 unit tests across 20 test files:

| Test File | Count | Coverage Area |
|---|---|---|
| BarometricAltitudeTests | 6 | Altitude source selection, barometer defaults |
| ContinuousCalibratorTests | — | LiDAR→neural calibration, inverse depth fitting |
| DEMRaycastEstimatorTests | — | Ray-terrain intersection, confidence tiers |
| DepthKalmanFilterTests | — | Kalman predict/update, motion adaptation |
| DepthSourceConfidenceTests | — | All 6 confidence curves, neural hard cap at 50m, DEM long-range |
| DisagreementPenaltyTests | — | Source outlier suppression logic |
| GeometricRangeEstimatorTests | — | Ground-plane model, slope penalty |
| IMUDepthPredictorTests | — | Motion-based prediction |
| InclinationCorrectorTests | — | Cosine correction, angle formatting |
| MonteCarloFusionTests | 10 | Full-pipeline semantic selection: 10K CI smoke test + 1M deep analysis across 10 distance bands (0.3–2000m) |
| BallisticsSolverTests | 16 | G1 holdover vs published tables (.308/5.56/6.5CM), hold direction, monotonicity, cross-caliber, metric, edge cases |
| TerrainRoutingTests | 14 | DEM-primary terrain routing, pitch guard, ray direction math, uncertainty, depth zone brackets, edge cases |
| DepthZoneBracketTests | 23 | DepthZoneOverlay struct, disagreement detection, bracket activation, formatting, AppState integration |
| MotionAwareSmootherTests | — | Adaptive smoothing, discontinuity detection |
| ReticleGeometryTests | — | Mil-to-pixel conversion |
| SceneClassifierTests | 11 | Sky/ground/structure detection, rate limiting |
| SRTMTileCacheTests | — | SRTM tile parsing, elevation queries, caching |
| **SemanticSelectionTests** | 14 | Priority chain ordering, source switch detection, DEM priority in far-target mode, background hypothesis, neural hard cap, DepthSource/RangeOutput enum changes |
| **StadiametricRangingTests** | 10 | Pinhole formula at various ranges, zero-pixel edge case, target presets, focal length independence, pixel-error sensitivity |
| SRTMDownloadTests | — | SRTM region manager, tile download |
