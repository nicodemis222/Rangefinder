//
//  BallisticsSolver.swift
//  Rangefinder
//
//  Bullet drop and holdover compensation calculator.
//  G1 drag model with step-integrated point-mass trajectory solver
//  and configurable caliber profiles.
//
//  Ballistic Model (step-integrated point-mass):
//    1. Convert target distance to yards
//    2. March trajectory in small steps (1-yard increments)
//    3. At each step: compute drag deceleration from G1 model,
//       update velocity, accumulate time and gravity drop
//    4. Compute sight-line angle from zero distance (scope height + drop at zero)
//    5. Holdover = bullet drop below sight line at target distance
//
//  Reference data (168gr SMK .308 Win @ 2650 fps, 100yd zero):
//    200yd: ~3.6"  / 0.5 mil    500yd: ~52" / 2.9 mil
//    300yd: ~12.5" / 1.2 mil   1000yd: ~370" / 10.3 mil
//

import Foundation
import Combine

// MARK: - Ballistics Solver

/// Ballistics solver for calculating bullet drop and holdover compensation.
/// Supports multiple caliber profiles with G1 drag model.
final class BallisticsSolver: ObservableObject {

    // MARK: - Caliber Definitions

    /// Supported caliber profiles with ballistic data.
    enum Caliber: String, CaseIterable, Identifiable, Codable {
        case cal308 = ".308 Win"
        case cal556 = "5.56 NATO"
        case cal65CM = "6.5 CM"
        case cal300WM = ".300 Win Mag"
        case cal338NM = ".338 Norma Mag"
        case cal338LM = ".338 Lapua Mag"

        var id: String { rawValue }

        /// Ballistic coefficient (G1)
        var ballisticCoefficient: Double {
            switch self {
            case .cal308: return 0.462   // 168gr SMK
            case .cal556: return 0.304   // 77gr SMK
            case .cal65CM: return 0.564  // 140gr ELD Match
            case .cal300WM: return 0.533 // 190gr BTHP
            case .cal338NM: return 0.691 // 300gr Berger Hybrid
            case .cal338LM: return 0.691 // 300gr Berger Hybrid
            }
        }

        /// Muzzle velocity in feet per second
        var muzzleVelocity: Double {
            switch self {
            case .cal308: return 2650
            case .cal556: return 2750
            case .cal65CM: return 2700
            case .cal300WM: return 2900
            case .cal338NM: return 2650
            case .cal338LM: return 2750
            }
        }

        /// Bullet weight in grains
        var bulletWeight: Double {
            switch self {
            case .cal308: return 168
            case .cal556: return 77
            case .cal65CM: return 140
            case .cal300WM: return 190
            case .cal338NM: return 300
            case .cal338LM: return 300
            }
        }

        /// Sight height over bore in inches
        var sightHeight: Double {
            return 1.5  // Standard AR/bolt rifle height
        }

        /// Short display name for HUD
        var shortName: String {
            switch self {
            case .cal308: return ".308"
            case .cal556: return "5.56"
            case .cal65CM: return "6.5CM"
            case .cal300WM: return ".300WM"
            case .cal338NM: return ".338NM"
            case .cal338LM: return ".338LM"
            }
        }
    }

    // MARK: - Published State

    @Published var selectedCaliber: Caliber {
        didSet { UserDefaults.standard.set(selectedCaliber.rawValue, forKey: "ballisticsCaliber") }
    }

    @Published var zeroDistance: Double {
        didSet { UserDefaults.standard.set(zeroDistance, forKey: "ballisticsZero") }
    }

    @Published var isEnabled: Bool {
        didSet { UserDefaults.standard.set(isEnabled, forKey: "ballisticsEnabled") }
    }

    // MARK: - Initialization

    init() {
        // Restore persisted settings
        if let savedCaliber = UserDefaults.standard.string(forKey: "ballisticsCaliber"),
           let caliber = Caliber(rawValue: savedCaliber) {
            self.selectedCaliber = caliber
        } else {
            self.selectedCaliber = .cal308
        }

        let savedZero = UserDefaults.standard.double(forKey: "ballisticsZero")
        self.zeroDistance = savedZero > 0 ? savedZero : 100

        self.isEnabled = UserDefaults.standard.object(forKey: "ballisticsEnabled") != nil
            ? UserDefaults.standard.bool(forKey: "ballisticsEnabled")
            : false  // Disabled by default
    }

    // MARK: - Public Methods

    /// Calculate holdover for a given distance.
    /// - Parameters:
    ///   - targetDistance: Distance to target in meters (always meters from RangingEngine)
    ///   - useMetric: Whether zero distance setting is in meters (true) or yards (false)
    /// - Returns: `HoldoverResult` with holdover in inches, cm, and mils
    func calculateHoldover(targetDistance: Double, useMetric: Bool) -> HoldoverResult {
        guard isEnabled, targetDistance > 0 else { return .zero }

        // Convert everything to yards for internal calculation
        let targetYards = targetDistance * 1.09361  // targetDistance is always meters
        let zeroYards = useMetric ? zeroDistance * 1.09361 : zeroDistance

        guard targetYards > 1, zeroYards > 1 else { return .zero }

        // Calculate pure gravity drop (below bore line) at zero and target distances
        let dropAtZero = calculateGravityDrop(distanceYards: zeroYards)
        let dropAtTarget = calculateGravityDrop(distanceYards: targetYards)

        // Sight-line geometry:
        //
        //   Scope sits sightHeight (h) above bore. At zero distance (Z),
        //   the bullet crosses the sight line, so the sight line angle is:
        //     angle = atan((h + drop(Z)) / Z_feet)
        //
        //   At target distance (D), the sight line is at height:
        //     sightLineHeight = h - tan(angle) × D_feet
        //
        //   Bullet is at height: -drop(D) below bore
        //   Holdover = distance bullet is below sight line (positive = hold high)
        //
        //   Simplified (small angle, linear):
        //     holdover = drop(D) - drop(Z)×(D/Z) + h×(1 - D/Z)
        //
        //   Or equivalently: holdover = drop(D) + h - (drop(Z) + h)×(D/Z)
        //
        let sightHeight = selectedCaliber.sightHeight
        let ratio = targetYards / zeroYards
        let holdoverInches = dropAtTarget + sightHeight - (dropAtZero + sightHeight) * ratio

        // Convert to milliradians:
        // 1 mil subtends 3.6 inches at 100 yards → at D yards: 1 mil = D × 0.036 inches
        let milSubtension = targetYards * 0.036
        let holdoverMils = milSubtension > 0 ? abs(holdoverInches) / milSubtension : 0

        // holdoverInches > 0 means bullet is below sight line → hold HIGH
        // holdoverInches < 0 means bullet is above sight line → hold LOW (inside zero)
        return HoldoverResult(
            holdoverInches: holdoverInches,
            holdoverMils: holdoverMils,
            holdHigh: holdoverInches > 0,
            useMetric: useMetric
        )
    }

    // MARK: - G1 Step-Integrated Trajectory

    /// Standard G1 drag coefficient table: (Mach number, Cd) pairs.
    /// Derived from the Ingalls/Mayevski G1 standard projectile drag function.
    /// This is the industry-standard reference used by all ballistic calculators.
    private static let g1DragTable: [(mach: Double, cd: Double)] = [
        (0.00, 0.2629),
        (0.05, 0.2558), (0.10, 0.2487), (0.15, 0.2413), (0.20, 0.2344),
        (0.25, 0.2278), (0.30, 0.2214), (0.35, 0.2155), (0.40, 0.2104),
        (0.45, 0.2061), (0.50, 0.2032), (0.55, 0.2020), (0.60, 0.2034),
        (0.65, 0.2165), (0.70, 0.2230), (0.75, 0.2313), (0.80, 0.2417),
        (0.85, 0.2546), (0.875, 0.2691), (0.90, 0.2853), (0.925, 0.3036),
        (0.95, 0.3237), (0.975, 0.3453), (1.00, 0.3632), (1.025, 0.3809),
        (1.05, 0.3980), (1.075, 0.4144), (1.10, 0.4299), (1.125, 0.4444),
        (1.15, 0.4578), (1.175, 0.4700), (1.20, 0.4811), (1.225, 0.4905),
        (1.25, 0.4983), (1.30, 0.5105), (1.35, 0.5190), (1.40, 0.5245),
        (1.45, 0.5276), (1.50, 0.5289), (1.55, 0.5288), (1.60, 0.5276),
        (1.65, 0.5255), (1.70, 0.5228), (1.75, 0.5196), (1.80, 0.5160),
        (1.85, 0.5122), (1.90, 0.5082), (1.95, 0.5040), (2.00, 0.4998),
        (2.05, 0.4955), (2.10, 0.4913), (2.15, 0.4871), (2.20, 0.4829),
        (2.25, 0.4787), (2.30, 0.4747), (2.35, 0.4707), (2.40, 0.4668),
        (2.45, 0.4630), (2.50, 0.4593), (2.60, 0.4521), (2.70, 0.4452),
        (2.80, 0.4386), (2.90, 0.4322), (3.00, 0.4261), (3.50, 0.3985),
        (4.00, 0.3750), (4.50, 0.3547), (5.00, 0.3374),
    ]

    /// Speed of sound at sea level, standard atmosphere (ft/s)
    private static let speedOfSound = 1125.0

    /// G1 drag coefficient: standard atmosphere, sea-level constant.
    /// dragCoeff × Cd(M) × v² / BC gives deceleration in ft/s².
    ///
    /// Derivation:
    ///   F_drag = 0.5 × ρ × v² × Cd × A_ref
    ///   a_drag = F_drag / m_ref = 0.5 × ρ × v² × Cd × A_ref / m_ref
    ///   BC = m / (Cd_form × A) = sectionalDensity / formFactor
    ///   For G1 reference: m_ref = 1 lb, d_ref = 1 inch, A_ref = π/4 in²
    ///   ρ_sealevel = 0.0765 lb/ft³
    ///
    /// The net drag constant that relates BC, Cd(Mach), and v² to
    /// deceleration is empirically verified: with this constant,
    /// .308 168gr SMK at 2650fps produces holdover values matching
    /// published ballistic tables within ±15%.
    private static let dragConstant = 0.0002087

    /// Interpolate G1 drag coefficient from Mach number.
    private static func g1Cd(mach: Double) -> Double {
        let table = g1DragTable
        // Clamp to table range
        if mach <= table.first!.mach { return table.first!.cd }
        if mach >= table.last!.mach { return table.last!.cd }

        // Binary search for bracket
        var lo = 0
        var hi = table.count - 1
        while lo < hi - 1 {
            let mid = (lo + hi) / 2
            if table[mid].mach <= mach {
                lo = mid
            } else {
                hi = mid
            }
        }

        // Linear interpolation
        let t = (mach - table[lo].mach) / (table[hi].mach - table[lo].mach)
        return table[lo].cd + t * (table[hi].cd - table[lo].cd)
    }

    /// Calculate pure gravity drop below the bore line using step-integrated
    /// G1 point-mass trajectory solver with full Mach-dependent drag table.
    ///
    /// Marches the trajectory in 1-yard increments, computing drag deceleration
    /// at each step using the G1 drag coefficient table (Cd vs Mach).
    /// Returns total gravity drop in inches (positive downward) at the given range.
    private func calculateGravityDrop(distanceYards: Double) -> Double {
        let bc = selectedCaliber.ballisticCoefficient
        let mv = selectedCaliber.muzzleVelocity

        // Step size in yards
        let stepYards = 1.0

        var velocity = mv              // Current velocity (ft/s)
        var timeOfFlight = 0.0         // Accumulated time (seconds)
        var distanceTraveled = 0.0     // Distance traveled (yards)

        // March trajectory in steps
        while distanceTraveled < distanceYards {
            // Remaining distance — use partial step at end
            let remainingYards = distanceYards - distanceTraveled
            let currentStepYards = min(stepYards, remainingYards)
            let currentStepFeet = currentStepYards * 3.0

            // Time for this step at current velocity
            guard velocity > 100 else { break }  // Floor to prevent divide-by-zero
            let dt = currentStepFeet / velocity

            // G1 drag deceleration: a = dragConstant × Cd(Mach) × v² / BC
            let mach = velocity / Self.speedOfSound
            let cd = Self.g1Cd(mach: mach)
            let deceleration = Self.dragConstant * cd * velocity * velocity / bc

            // Update velocity (Euler integration)
            velocity -= deceleration * dt
            velocity = max(velocity, 100)  // Subsonic floor

            // Accumulate time
            timeOfFlight += dt
            distanceTraveled += currentStepYards
        }

        // Gravity drop: d = 0.5 × g × t² (in feet, then convert to inches)
        let gravityDropFeet = 0.5 * 32.174 * timeOfFlight * timeOfFlight
        let gravityDropInches = gravityDropFeet * 12.0

        return gravityDropInches
    }
}

// MARK: - Holdover Result

struct HoldoverResult {
    let holdoverInches: Double      // Signed holdover in inches (positive = bullet below sight line)
    let holdoverMils: Double        // Absolute holdover in milliradians for reticle
    let holdHigh: Bool              // true = hold high (aim above target), false = hold low
    let useMetric: Bool             // Whether user display is metric

    /// Holdover in centimeters (absolute)
    var holdoverCM: Double {
        abs(holdoverInches) * 2.54
    }

    /// Formatted inches value
    var displayInches: String {
        let v = abs(holdoverInches)
        if v < 1 { return String(format: "%.1f", v) }
        if v < 100 { return String(format: "%.1f", v) }
        return String(format: "%.0f", v)
    }

    /// Formatted centimeters value
    var displayCM: String {
        let v = holdoverCM
        if v < 1 { return String(format: "%.1f", v) }
        if v < 100 { return String(format: "%.1f", v) }
        return String(format: "%.0f", v)
    }

    /// Formatted mil value
    var displayMils: String {
        if holdoverMils < 0.1 {
            return "0.0"
        } else if holdoverMils < 10 {
            return String(format: "%.1f", holdoverMils)
        } else {
            return String(format: "%.0f", holdoverMils)
        }
    }

    /// Primary value in user's preferred unit (inches or cm)
    var displayValue: String {
        useMetric ? displayCM : displayInches
    }

    /// Primary unit label
    var unitLabel: String {
        useMetric ? "CM" : "IN"
    }

    /// Direction text
    var directionText: String {
        holdHigh ? "UP" : "DN"
    }

    /// Full description
    var description: String {
        "\(displayMils) MIL \(directionText)"
    }

    /// Is this a meaningful holdover (worth displaying)?
    var isSignificant: Bool {
        holdoverMils >= 0.1
    }

    static let zero = HoldoverResult(holdoverInches: 0, holdoverMils: 0, holdHigh: false, useMetric: false)
}
