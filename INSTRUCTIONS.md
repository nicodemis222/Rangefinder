# MilDot Rangefinder — Instructions

## Overview

MilDot Rangefinder is an iOS application that turns an iPhone into a multi-source digital rangefinder. It fuses LiDAR, neural depth estimation, geometric ground-plane ranging, DEM terrain ray-casting, and object-size ranging into a single unified depth field with confidence-weighted output from 0 to 2000 meters.

The interface presents a configurable reticle overlay (mil-dot, crosshair, or rangefinder) on the live camera feed with real-time range readout, confidence indicators, source blend visualization, compass bearing, operator guidance coaching, inclination correction, and optional ballistic holdover computation.

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

There are 243 unit tests covering all depth sources, confidence curves, Kalman filtering, calibration, scene classification, geometric estimation, DEM ray-casting, terrain routing, depth zone brackets, and Monte Carlo fusion validation.

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
│   ├── UnifiedDepthField.swift     Fusion engine (6 sources → 1 estimate)
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
    ├── RangefinderView.swift       Main camera + HUD view + compass bearing chip
    ├── HUDOverlayView.swift        Tactical heads-up display overlay
    ├── RangeDisplayView.swift      Primary range readout
    ├── ConfidenceBadge.swift       Color-coded confidence indicator
    ├── HoldoverIndicator.swift     Ballistic holdover display
    ├── OperatorGuidanceView.swift  Stability bar + coaching hint chips
    ├── SettingsView.swift          Configuration panel + reticle style picker
    ├── TutorialView.swift          7-page tactical onboarding tutorial
    └── Theme.swift                 MIL-STD-3009 color palette
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

**HUD Elements (Top Bar Chips):**
- **MAG**: current magnification (0.5x–25x)
- **HDG**: compass bearing + cardinal direction (e.g., 045°NE) — visible when GPS is active
- **ELEV**: pitch angle with color coding by severity
- **Ballistics chip**: caliber + zero distance (when enabled, tappable for quick caliber/zero selection)
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
- **Depth Sources**: individual enable/disable for geometric and DEM sources
- **System Info**: model status, GPS quality, barometer status, detection capabilities

### Depth Sources

The app automatically selects and blends depth sources based on distance:

| Range | Primary Sources | Notes |
|---|---|---|
| 0–5m | LiDAR | Direct time-of-flight measurement |
| 5–15m | Neural + LiDAR tail | Calibrated DepthAnythingV2 with LiDAR overlap |
| 15–50m | Neural + Geometric | Calibrated neural extrapolation + ground-plane model |
| 50–200m | **DEM (terrain routed)** or fusion | DEM is authoritative for terrain; fusion used when objects detected |
| 200–2000m | **DEM (terrain routed)** + Object | GPS/terrain-based ranging; object detection corroborates when available |

**Terrain Routing:** When GPS/DEM is available and no discrete object is detected at the crosshair, the DEM ray-cast answer is used directly — other sources are shown in the depth zone brackets but don't dilute the terrain distance. This is critical for scenarios like ranging to a mountain over a rock wall, where neural/LiDAR see the foreground obstacle but DEM correctly traces the terrain at 1600m.

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

243 unit tests across 17 test files:

| Test File | Count | Coverage Area |
|---|---|---|
| BarometricAltitudeTests | 6 | Altitude source selection, barometer defaults |
| ContinuousCalibratorTests | — | LiDAR→neural calibration, inverse depth fitting |
| DEMRaycastEstimatorTests | — | Ray-terrain intersection, confidence tiers |
| DepthKalmanFilterTests | — | Kalman predict/update, motion adaptation |
| DepthSourceConfidenceTests | — | All 5 confidence curves, calibration quality, DEM long-range |
| DisagreementPenaltyTests | — | Source outlier suppression logic |
| GeometricRangeEstimatorTests | — | Ground-plane model, slope penalty |
| IMUDepthPredictorTests | — | Motion-based prediction |
| InclinationCorrectorTests | — | Cosine correction, angle formatting |
| MonteCarloFusionTests | 10 | Full-pipeline fusion with terrain routing: 10K CI smoke test + 1M deep analysis across 10 distance bands (0.3–2000m), 5.2% mean error |
| BallisticsSolverTests | 16 | G1 holdover vs published tables (.308/5.56/6.5CM), hold direction, monotonicity, cross-caliber, metric, edge cases |
| TerrainRoutingTests | 14 | DEM-primary terrain routing, pitch guard, ray direction math, uncertainty, depth zone brackets, edge cases |
| DepthZoneBracketTests | 23 | DepthZoneOverlay struct, disagreement detection, bracket activation, formatting, AppState integration |
| MotionAwareSmootherTests | — | Adaptive smoothing, discontinuity detection |
| ReticleGeometryTests | — | Mil-to-pixel conversion |
| SceneClassifierTests | 11 | Sky/ground/structure detection, rate limiting |
| SRTMTileCacheTests | — | SRTM tile parsing, elevation queries, caching |
