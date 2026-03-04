//
//  AppState.swift
//  Rangefinder
//
//  Global MVVM state wiring camera -> depth -> ranging -> UI.
//

import SwiftUI
import Combine
import CoreLocation
import os

@MainActor
class AppState: ObservableObject {
    // MARK: - Published State

    @Published var currentRange: RangeOutput = .none
    @Published var pitchDegrees: Double = 0.0
    @Published var headingDegrees: Double = 0.0
    @Published var isSessionRunning = false
    @Published var sessionError: String?
    @Published var displayUnit: UnitLength = .yards {
        didSet { UserDefaults.standard.set(displayUnit == .yards ? "yards" : "meters", forKey: "displayUnit") }
    }
    @Published var reticleConfig: ReticleConfiguration = .default
    @Published var isModelsLoaded = false
    @Published var showSettings = false
    @Published var currentHoldover: HoldoverResult?
    @Published var backgroundRange: RangeOutput = .none
    @Published var isStadiametricMode: Bool = false
    @Published var stadiametricTargetSize: Double = 1.8
    @Published var stadiametricPixelSize: Double = 0
    @Published var showMapPiP: Bool = false
    @Published var demHitCoordinate: CLLocationCoordinate2D?
    @Published var semanticDecision: SemanticSourceDecision = .none
    @Published var sceneRangeResult: SceneRangeResult = .empty
    @Published var cameraHeight: Float = AppConfiguration.defaultCameraHeight {
        didSet {
            UserDefaults.standard.set(cameraHeight, forKey: "cameraHeight")
            depthField.geometricEstimator.cameraHeight = cameraHeight
        }
    }

    // MARK: - Depth Zone Overlay (for reticle)

    /// Computed overlay for the reticle's depth zone brackets.
    /// Always shows current depth scene — crosshair reading vs DEM reading.
    /// When these disagree, the brackets visually separate to alert the operator.
    var depthZoneOverlay: DepthZoneOverlay {
        let analysis = depthField.latestBimodalAnalysis
        let crosshairM = Float(currentRange.isValid
            ? currentRange.adjustedRange.converted(to: .meters).value
            : 0)
        let demEstimate = depthField.latestDEMEstimate
        let hasDEM = demEstimate != nil && (demEstimate?.confidence ?? 0) > 0.05

        return DepthZoneOverlay(
            isBimodal: analysis.isBimodal,
            crosshairDepthM: crosshairM,
            demDepthM: demEstimate?.distanceMeters ?? 0,
            hasDEM: hasDEM,
            activeZone: .far,
            nearPeakM: analysis.nearPeakM,
            farPeakM: analysis.farPeakM
        )
    }

    // MARK: - Managers

    let cameraManager = CameraManager()
    let inclinationManager = InclinationManager()
    let performanceMonitor = PerformanceMonitor()
    let ballisticsSolver = BallisticsSolver()
    let locationManager = LocationManager()
    let guidanceEngine = OperatorGuidanceEngine()
    let regionManager = SRTMRegionManager()

    // MARK: - Depth + Ranging

    let depthField = UnifiedDepthField()
    let tileCache = SRTMTileCache()
    private(set) lazy var rangingEngine = RangingEngine(
        depthField: depthField,
        inclinationManager: inclinationManager
    )

    // MARK: - Private

    private var cancellables = Set<AnyCancellable>()
    private var frameProcessingTask: Task<Void, Never>?
    private var hasStartedSession = false

    // MARK: - Initialization

    init() {
        // Restore persisted settings
        if let savedUnit = UserDefaults.standard.string(forKey: "displayUnit") {
            displayUnit = savedUnit == "meters" ? .meters : .yards
        }
        if let savedHeight = UserDefaults.standard.object(forKey: "cameraHeight") as? Float {
            cameraHeight = savedHeight
            depthField.geometricEstimator.cameraHeight = savedHeight
        }
        // Always use far-target mode — the engine auto-detects bimodal scenes
        // and prioritizes DEM terrain ranging over foreground objects.
        depthField.targetPriority = .far
        setupBindings()
    }

    private func setupBindings() {
        // Forward camera session state
        cameraManager.$isSessionRunning
            .assign(to: &$isSessionRunning)

        cameraManager.$sessionError
            .assign(to: &$sessionError)

        // Forward ranging output
        rangingEngine.$currentRange
            .assign(to: &$currentRange)

        // Forward background hypothesis
        rangingEngine.$backgroundRange
            .assign(to: &$backgroundRange)

        // Forward semantic decision
        depthField.$semanticDecision
            .assign(to: &$semanticDecision)

        // Forward scene range result (multi-point ranging)
        rangingEngine.$sceneRangeResult
            .assign(to: &$sceneRangeResult)

        // Forward pitch and heading from inclination manager
        inclinationManager.$pitchDegrees
            .assign(to: &$pitchDegrees)

        inclinationManager.$headingDegrees
            .assign(to: &$headingDegrees)

        // Compute holdover when range changes
        $currentRange
            .map { [weak self] range -> HoldoverResult? in
                guard let self = self, range.isValid else { return nil }
                let distanceM = range.adjustedRange.converted(to: .meters).value
                let useMetric = self.displayUnit == .meters
                let result = self.ballisticsSolver.calculateHoldover(
                    targetDistance: distanceM,
                    useMetric: useMetric
                )
                return result.isSignificant ? result : nil
            }
            .assign(to: &$currentHoldover)

        // Update DEM hit coordinate for map display
        $currentRange
            .map { [weak self] _ -> CLLocationCoordinate2D? in
                self?.depthField.latestDEMEstimate?.hitCoordinate
            }
            .assign(to: &$demHitCoordinate)

        // Process camera frames through the full pipeline
        cameraManager.frameSubject
            .receive(on: DispatchQueue.main)
            .sink { [weak self] frameData in
                guard let self = self else { return }
                self.handleFrame(frameData)
            }
            .store(in: &cancellables)
    }

    // MARK: - Frame Processing

    /// Tracks whether the pipeline is currently processing a frame.
    /// We only allow ONE frame in-flight at a time. If a new frame arrives
    /// while the previous is still processing, we silently drop it.
    /// This prevents task pile-up since Task.cancel() does NOT interrupt
    /// in-flight CoreML inference — the work runs to completion regardless.
    private var isFrameInFlight = false

    private func handleFrame(_ frameData: FrameData) {
        // Update pitch from AR frame as backup (instant, always runs)
        inclinationManager.updateFromARFrame(pitchRadians: frameData.pitchRadians)

        // Record for FPS tracking (instant, always runs)
        performanceMonitor.recordFrame()

        // ALWAYS feed camera pose to IMU predictor — even on dropped frames.
        // This keeps the motion model accurate during inference gaps, so the
        // Kalman filter can predict depth forward while neural inference runs.
        rangingEngine.feedIMU(
            cameraTransform: frameData.cameraPose,
            timestamp: frameData.timestamp
        )

        // Update operator guidance engine with current motion + sensor state
        updateGuidanceEngine(timestamp: frameData.timestamp)

        // Drop frame if previous is still in-flight.
        // This is better than cancel+relaunch because CoreML inference
        // can't actually be interrupted — cancelling just orphans work.
        guard !isFrameInFlight else { return }

        isFrameInFlight = true
        frameProcessingTask = Task { [weak self] in
            guard let self = self else { return }
            await self.rangingEngine.processFrame(frameData)
            self.isFrameInFlight = false
        }
    }

    /// Feed current sensor state into the operator guidance engine.
    /// Runs every frame (~30 Hz) — the engine throttles internally.
    private func updateGuidanceEngine(timestamp: TimeInterval) {
        // Feed angular velocity for stability analysis
        guidanceEngine.update(
            angularVelocity: inclinationManager.angularVelocity,
            timestamp: timestamp
        )

        // Feed current range for reading-lock detection
        if currentRange.isValid {
            guidanceEngine.feedRange(
                rangeM: currentRange.adjustedRange.converted(to: .meters).value,
                timestamp: timestamp
            )
            guidanceEngine.currentRangeM = currentRange.adjustedRange.converted(to: .meters).value
            guidanceEngine.currentConfidence = currentRange.confidence
        }

        // Update external sensor state
        guidanceEngine.hasGPSFix = locationManager.hasValidFix
        guidanceEngine.isGPSAuthorized = locationManager.isAuthorized
        guidanceEngine.gpsAccuracy = Float(locationManager.horizontalAccuracy)
        guidanceEngine.headingAccuracy = 10  // Approximate; CMMotionManager doesn't expose this directly

        // Calibration state
        guidanceEngine.calibrationAge = depthField.calibrator.calibrationAge(currentTimestamp: timestamp)
        guidanceEngine.calibrationConfidence = depthField.calibrator.confidence
        guidanceEngine.calibrationSampleCount = depthField.calibrator.calibration.sampleCount

        // Range quality state
        guidanceEngine.isNeuralUncorroborated = depthField.isNeuralUncorroborated
        guidanceEngine.isDEMAvailable = depthField.latestDEMEstimate != nil
        guidanceEngine.isDEMDownloading = regionManager.activeDownloadRegion != nil

        // Terrain coverage state for terrainRangeLimited hint
        guidanceEngine.isDEMFarField = (depthField.latestDEMEstimate?.distanceMeters ?? 0) > 200
        guidanceEngine.isLSTMode = true
        guidanceEngine.cameraPitchDegrees = pitchDegrees
        guidanceEngine.isBallisticsEnabled = ballisticsSolver.isEnabled
    }

    // MARK: - Lifecycle

    func startSession() async {
        guard !hasStartedSession else { return }
        hasStartedSession = true

        // Wire performance monitor for thermal throttling
        rangingEngine.performanceMonitor = performanceMonitor
        rangingEngine.locationManager = locationManager

        // Start camera
        cameraManager.setupSession()

        // Start motion tracking
        inclinationManager.startUpdates()

        // Start GPS for DEM ray-casting
        locationManager.requestAuthorization()
        locationManager.startUpdates()

        // Create DEM ray-cast estimator and wire to depth field
        let demEstimator = DEMRaycastEstimator(tileCache: tileCache)
        depthField.demEstimator = demEstimator

        // Scan for installed SRTM tiles
        regionManager.scanInstalledTiles()

        // Load ML models
        await rangingEngine.loadModels()
        isModelsLoaded = true

        Logger.ui.info("App session started, models loaded, DEM ready")
    }

    func pauseSession() {
        guard hasStartedSession else { return }
        cameraManager.pauseSession()
        inclinationManager.stopUpdates()
        locationManager.stopUpdates()
        frameProcessingTask?.cancel()
        isFrameInFlight = false
    }

    func resumeSession() {
        guard hasStartedSession else { return }
        cameraManager.resumeSession()
        inclinationManager.startUpdates()
        locationManager.startUpdates()
    }

    // MARK: - Stadiametric Mode

    func toggleStadiametricMode() {
        isStadiametricMode.toggle()
        if !isStadiametricMode {
            // Exiting stadia mode — clear the input so semantic selector
            // falls through to automated sources.
            depthField.stadiametricInput = nil
            stadiametricPixelSize = 0
        }
    }

    func updateStadiametricInput(pixelSize: Double) {
        stadiametricPixelSize = pixelSize
        guard isStadiametricMode, pixelSize > 0 else {
            depthField.stadiametricInput = nil
            return
        }
        // Focal length in pixels for the pinhole formula.
        // iPhone 15 Pro main camera (~26mm) at 1920x1440 ≈ 2160px.
        // TODO: Read from latest FrameData intrinsics for zoom-aware value.
        let focalLength: Double = 2160.0
        depthField.stadiametricInput = StadiametricInput(
            knownSizeMeters: stadiametricTargetSize,
            pixelSize: pixelSize,
            focalLengthPixels: focalLength
        )
    }

}
