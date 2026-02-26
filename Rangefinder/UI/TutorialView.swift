//
//  TutorialView.swift
//  Rangefinder
//
//  SYSTEM BRIEFING — tactical onboarding.
//  Two tutorial tracks: GENERAL RANGING and GOLF MODE.
//  Terse, technical wording. Military aesthetic.
//

import SwiftUI

// MARK: - Tutorial Category

enum TutorialCategory: String, CaseIterable, Identifiable {
    case general = "GENERAL"
    case golf = "GOLF"

    var id: String { rawValue }

    var label: String {
        switch self {
        case .general: return "GENERAL RANGING"
        case .golf: return "GOLF MODE"
        }
    }

    var icon: String {
        switch self {
        case .general: return "scope"
        case .golf: return "flag.fill"
        }
    }

    var subtitle: String {
        switch self {
        case .general: return "FULL SYSTEM BRIEFING"
        case .golf: return "PIN DISTANCE WORKFLOW"
        }
    }

    var pages: [TutorialPage] {
        switch self {
        case .general: return TutorialPage.generalPages
        case .golf: return TutorialPage.golfPages
        }
    }
}

// MARK: - Tutorial Page Model

struct TutorialPage: Identifiable {
    let id = UUID()
    let icon: String
    let iconColor: Color
    let title: String
    let subtitle: String
    let features: [TutorialFeature]
}

struct TutorialFeature: Identifiable {
    let id = UUID()
    let icon: String
    let text: String
    let color: Color
}

// MARK: - General Ranging Tutorial Content

extension TutorialPage {
    /// Legacy accessor — returns general pages for backward compatibility
    static let pages: [TutorialPage] = generalPages

    static let generalPages: [TutorialPage] = [
        TutorialPage(
            icon: "scope",
            iconColor: Theme.milGreen,
            title: "MILDOT RANGEFINDER",
            subtitle: "PASSIVE RANGING SYSTEM",
            features: [
                TutorialFeature(icon: "brain", text: "5-SOURCE FUSION: LIDAR + AI + GEO + DEM + OBJ", color: Theme.milGreen),
                TutorialFeature(icon: "sensor.fill", text: "LIDAR AUTO-CALIBRATION — NO SETUP REQ", color: Theme.milGreen),
                TutorialFeature(icon: "ruler", text: "EFFECTIVE RANGE 0-2000M, INCL CORRECTED", color: Theme.milAmber),
            ]
        ),
        TutorialPage(
            icon: "waveform.path.ecg",
            iconColor: Theme.milGreen,
            title: "SENSOR FUSION",
            subtitle: "UNIFIED DEPTH FIELD",
            features: [
                TutorialFeature(icon: "sensor.fill", text: "LIDAR: 0-5M — GROUND TRUTH CALIBRATION", color: Theme.milGreen),
                TutorialFeature(icon: "brain.head.profile", text: "NEURAL AI: 5-150M — CALIBRATED ESTIMATION", color: Theme.milAmber),
                TutorialFeature(icon: "mountain.2.fill", text: "DEM TERRAIN: 50-2000M — SRTM RAY-CAST", color: Theme.confidenceLow),
                TutorialFeature(icon: "viewfinder", text: "OBJECT DET: 30-1000M — KNOWN-SIZE RANGING", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "scope",
            iconColor: Theme.milGreen,
            title: "RETICLE OPTIONS",
            subtitle: "CONFIGURABLE STYLES",
            features: [
                TutorialFeature(icon: "circle.grid.cross", text: "MIL-DOT: NATO STANDARD, FFP SCALED", color: Theme.milGreen),
                TutorialFeature(icon: "plus", text: "CROSSHAIR: CLEAN — MAXIMUM TARGET CLARITY", color: Theme.milAmber),
                TutorialFeature(icon: "viewfinder", text: "RANGEFINDER: DUPLEX + BRACKET MARKS", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "hand.pinch",
            iconColor: Theme.milGreen,
            title: "MAGNIFICATION",
            subtitle: "OPTICAL + DIGITAL ZOOM",
            features: [
                TutorialFeature(icon: "camera.aperture", text: "0.5X ULTRAWIDE TO 8X OPTICAL TELEPHOTO", color: Theme.milGreen),
                TutorialFeature(icon: "arrow.up.backward.and.arrow.down.forward", text: "UP TO 25X WITH DIGITAL ZOOM", color: Theme.milAmber),
                TutorialFeature(icon: "scope", text: "RETICLE GROWS WITH ZOOM — STAYS CALIBRATED", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "display",
            iconColor: Theme.milGreen,
            title: "HUD DISPLAY",
            subtitle: "READING THE RANGE OUTPUT",
            features: [
                TutorialFeature(icon: "target", text: "RANGE IN YDS/M — CONFIDENCE COLOR CODED", color: Theme.milGreen),
                TutorialFeature(icon: "location.north.line", text: "COMPASS BEARING + ELEVATION ANGLE", color: Theme.milAmber),
                TutorialFeature(icon: "chart.bar.fill", text: "SOURCE BLEND BAR — SEE SENSOR CONTRIBUTIONS", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "hand.raised",
            iconColor: Theme.milGreen,
            title: "OPERATOR GUIDANCE",
            subtitle: "REAL-TIME COACHING",
            features: [
                TutorialFeature(icon: "lock.fill", text: "STABILITY BAR — SHOWS DEVICE HOLD QUALITY", color: Theme.milGreen),
                TutorialFeature(icon: "circle.circle", text: "CAPTURE WINDOW — OPTIMAL MOMENT TO READ", color: Theme.milAmber),
                TutorialFeature(icon: "clock.arrow.circlepath", text: "CAL STATUS — WALK NEAR OBJECTS TO REFRESH", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "exclamationmark.triangle",
            iconColor: Theme.milAmber,
            title: "OPERATING NOTES",
            subtitle: "MAXIMIZE ACCURACY",
            features: [
                TutorialFeature(icon: "hand.raised", text: "BRACE DEVICE: LEAN ON WALL/TREE/VEHICLE", color: Theme.milGreen),
                TutorialFeature(icon: "lungs", text: "CAPTURE AT EXHALE PAUSE — NATURAL STEADY POINT", color: Theme.milAmber),
                TutorialFeature(icon: "number.circle", text: "LONG RANGE: TAKE MULTIPLE READINGS, AVERAGE", color: Theme.milGreen),
                TutorialFeature(icon: "figure.walk", text: "WALK PAST OBJECTS TO REFRESH LIDAR CALIBRATION", color: Theme.milAmber),
            ]
        ),
    ]
}

// MARK: - Golf Tutorial Content

extension TutorialPage {
    static let golfPages: [TutorialPage] = [
        TutorialPage(
            icon: "flag.fill",
            iconColor: Theme.milAmber,
            title: "GOLF RANGEFINDER",
            subtitle: "PIN DISTANCE MODE",
            features: [
                TutorialFeature(icon: "scope", text: "STADIAMETRIC RANGING — BRACKET THE PIN", color: Theme.milGreen),
                TutorialFeature(icon: "ruler", text: "USGA PIN HEIGHT: 7 FT / 2.13M STANDARD", color: Theme.milAmber),
                TutorialFeature(icon: "checkmark.seal", text: "ACCURATE 40-250 YDS — NO SUBSCRIPTION REQ", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "1.circle.fill",
            iconColor: Theme.milGreen,
            title: "STEP 1: SELECT TARGET",
            subtitle: "SETTINGS > STADIAMETRIC",
            features: [
                TutorialFeature(icon: "gearshape", text: "OPEN SETTINGS — STADIAMETRIC RANGING SECTION", color: Theme.milGreen),
                TutorialFeature(icon: "flag.fill", text: "TAP GOLF PIN PRESET — SETS HEIGHT TO 2.13M", color: Theme.milAmber),
                TutorialFeature(icon: "scope", text: "SELECT RANGEFINDER OR CROSSHAIR RETICLE", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "2.circle.fill",
            iconColor: Theme.milGreen,
            title: "STEP 2: FRAME THE PIN",
            subtitle: "ZOOM + AIM AT FLAGSTICK",
            features: [
                TutorialFeature(icon: "hand.pinch", text: "PINCH ZOOM 5-8X — PIN MUST BE VISIBLE", color: Theme.milGreen),
                TutorialFeature(icon: "arrow.up.and.down", text: "CENTER CROSSHAIR ON PIN — BASE TO TOP", color: Theme.milAmber),
                TutorialFeature(icon: "lock.fill", text: "HOLD STEADY — WAIT FOR STABILITY INDICATOR", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "3.circle.fill",
            iconColor: Theme.milGreen,
            title: "STEP 3: BRACKET & READ",
            subtitle: "USE STADIA MARKS FOR RANGE",
            features: [
                TutorialFeature(icon: "arrow.up.and.down.square", text: "ALIGN STADIA BRACKET TO SPAN FULL PIN", color: Theme.milGreen),
                TutorialFeature(icon: "target", text: "RANGE DISPLAYS AUTOMATICALLY IN YDS/M", color: Theme.milAmber),
                TutorialFeature(icon: "number.circle", text: "TAKE 2-3 READINGS — USE AVERAGE FOR BEST RESULT", color: Theme.milGreen),
            ]
        ),
        TutorialPage(
            icon: "chart.bar.fill",
            iconColor: Theme.milAmber,
            title: "ACCURACY BY RANGE",
            subtitle: "EXPECTED PRECISION",
            features: [
                TutorialFeature(icon: "checkmark.circle", text: "40-100 YDS: ~2% ERROR — CLUB SELECTION GRADE", color: Theme.milGreen),
                TutorialFeature(icon: "checkmark.circle", text: "100-150 YDS: ~3-5% — RELIABLE APPROACH SHOTS", color: Theme.milGreen),
                TutorialFeature(icon: "exclamationmark.circle", text: "150-200 YDS: ~5-8% — PIN GETS SMALL, ZOOM IN", color: Theme.milAmber),
                TutorialFeature(icon: "exclamationmark.triangle", text: "200-250 YDS: ~8-12% — USE TELEPHOTO, BRACE", color: Theme.confidenceLow),
            ]
        ),
        TutorialPage(
            icon: "lightbulb.fill",
            iconColor: Theme.milAmber,
            title: "GOLF TIPS",
            subtitle: "MAXIMIZE PIN ACCURACY",
            features: [
                TutorialFeature(icon: "hand.raised", text: "BRACE PHONE ON CART OR BAG FOR STABILITY", color: Theme.milGreen),
                TutorialFeature(icon: "camera.aperture", text: "USE 5X+ ZOOM — BIGGER PIN = BETTER BRACKET", color: Theme.milAmber),
                TutorialFeature(icon: "eye", text: "RANGE THE STICK NOT THE FLAG — TOP OF POLE", color: Theme.milGreen),
                TutorialFeature(icon: "figure.walk", text: "WALK PAST CART PATH MARKERS TO CALIBRATE LIDAR", color: Theme.milAmber),
            ]
        ),
    ]
}

// MARK: - Tutorial View

struct TutorialView: View {
    let onComplete: () -> Void
    var initialCategory: TutorialCategory? = nil
    @State private var selectedCategory: TutorialCategory? = nil
    @State private var currentPage = 0

    private var activePages: [TutorialPage] {
        selectedCategory?.pages ?? []
    }

    var body: some View {
        ZStack {
            Color.black
                .ignoresSafeArea()

            if let category = selectedCategory {
                // Page view for selected category
                tutorialPagesView(category: category)
                    .transition(.asymmetric(
                        insertion: .move(edge: .trailing).combined(with: .opacity),
                        removal: .move(edge: .leading).combined(with: .opacity)
                    ))
            } else {
                // Category picker
                categoryPickerView
                    .transition(.opacity)
            }
        }
        .onAppear {
            if let initial = initialCategory {
                selectedCategory = initial
            }
        }
    }

    // MARK: - Category Picker

    private var categoryPickerView: some View {
        VStack(spacing: 0) {
            // Dismiss button
            HStack {
                Spacer()
                Button("DISMISS") {
                    onComplete()
                }
                .font(.system(size: 12, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milGreenDim)
                .padding(.trailing, 20)
                .padding(.top, 16)
            }

            Spacer()

            // Title
            Image(systemName: "scope")
                .font(.system(size: 56, weight: .light))
                .foregroundColor(Theme.milGreen)

            VStack(spacing: 8) {
                Text("SYSTEM BRIEFING")
                    .font(Theme.tutorialTitleFont)
                    .foregroundColor(Theme.milGreen)
                    .tracking(1)
                Text("SELECT TUTORIAL")
                    .font(Theme.tutorialBodyFont)
                    .foregroundColor(Theme.milGreenDim)
                    .tracking(0.5)
            }
            .padding(.top, 24)

            Spacer()
                .frame(height: 48)

            // Category cards
            VStack(spacing: 16) {
                ForEach(TutorialCategory.allCases) { category in
                    Button {
                        withAnimation(.easeInOut(duration: 0.3)) {
                            selectedCategory = category
                            currentPage = 0
                        }
                    } label: {
                        HStack(spacing: 16) {
                            Image(systemName: category.icon)
                                .font(.system(size: 28, weight: .light))
                                .foregroundColor(category == .golf ? Theme.milAmber : Theme.milGreen)
                                .frame(width: 44)

                            VStack(alignment: .leading, spacing: 4) {
                                Text(category.label)
                                    .font(.system(size: 16, weight: .bold, design: .monospaced))
                                    .foregroundColor(Theme.milGreen)
                                    .tracking(0.5)
                                Text(category.subtitle)
                                    .font(.system(size: 12, weight: .medium, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                                Text("\(category.pages.count) CARDS")
                                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                                    .foregroundColor(Theme.milGreenDim)
                            }

                            Spacer()

                            Image(systemName: "chevron.right")
                                .font(.system(size: 14, weight: .bold))
                                .foregroundColor(Theme.milGreenDim)
                        }
                        .padding(.horizontal, 20)
                        .padding(.vertical, 16)
                        .background(
                            RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                                .fill(Theme.panelBackground)
                                .overlay(
                                    RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                                        .stroke(
                                            category == .golf
                                                ? Theme.milAmber.opacity(0.4)
                                                : Theme.milGreen.opacity(0.4),
                                            lineWidth: Theme.borderWidth
                                        )
                                )
                        )
                    }
                }
            }
            .padding(.horizontal, 24)

            Spacer()
            Spacer()
        }
    }

    // MARK: - Tutorial Pages

    private func tutorialPagesView(category: TutorialCategory) -> some View {
        VStack(spacing: 0) {
            // Top bar: back + dismiss
            HStack {
                Button {
                    withAnimation(.easeInOut(duration: 0.3)) {
                        selectedCategory = nil
                        currentPage = 0
                    }
                } label: {
                    HStack(spacing: 4) {
                        Image(systemName: "chevron.left")
                            .font(.system(size: 11, weight: .bold))
                        Text("BACK")
                            .font(.system(size: 12, weight: .bold, design: .monospaced))
                    }
                    .foregroundColor(Theme.milGreenDim)
                }
                .padding(.leading, 20)
                .padding(.top, 16)

                Spacer()

                Button("DISMISS") {
                    onComplete()
                }
                .font(.system(size: 12, weight: .bold, design: .monospaced))
                .foregroundColor(Theme.milGreenDim)
                .padding(.trailing, 20)
                .padding(.top, 16)
            }

            // Page content
            TabView(selection: $currentPage) {
                ForEach(Array(category.pages.enumerated()), id: \.element.id) { index, page in
                    TutorialCardView(page: page)
                        .tag(index)
                }
            }
            .tabViewStyle(.page(indexDisplayMode: .always))
            .indexViewStyle(.page(backgroundDisplayMode: .always))

            // Bottom button
            Button(action: {
                if currentPage < category.pages.count - 1 {
                    withAnimation {
                        currentPage += 1
                    }
                } else {
                    onComplete()
                }
            }) {
                Text(currentPage < category.pages.count - 1 ? "NEXT" : "ACKNOWLEDGE")
                    .font(.system(size: 16, weight: .bold, design: .monospaced))
                    .tracking(2)
                    .foregroundColor(Theme.milGreen)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                    .background(
                        RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                            .fill(Theme.panelBackground)
                            .overlay(
                                RoundedRectangle(cornerRadius: Theme.hudCornerRadius)
                                    .stroke(Theme.milGreen.opacity(0.6), lineWidth: Theme.borderWidth)
                            )
                    )
            }
            .padding(.horizontal, 24)
            .padding(.bottom, 40)
        }
    }
}

// MARK: - Tutorial Card View

struct TutorialCardView: View {
    let page: TutorialPage

    var body: some View {
        VStack(spacing: 24) {
            Spacer()

            // Icon
            Image(systemName: page.icon)
                .font(.system(size: 56, weight: .light))
                .foregroundColor(page.iconColor)

            // Title & Subtitle
            VStack(spacing: 8) {
                Text(page.title)
                    .font(Theme.tutorialTitleFont)
                    .foregroundColor(Theme.milGreen)
                    .tracking(1)
                Text(page.subtitle)
                    .font(Theme.tutorialBodyFont)
                    .foregroundColor(Theme.milGreenDim)
                    .tracking(0.5)
            }

            // Features
            VStack(alignment: .leading, spacing: 14) {
                ForEach(page.features) { feature in
                    HStack(spacing: 12) {
                        Image(systemName: feature.icon)
                            .font(.system(size: 18))
                            .foregroundColor(feature.color)
                            .frame(width: 26)

                        Text(feature.text)
                            .font(.system(size: 13, weight: .medium, design: .monospaced))
                            .foregroundColor(Theme.milGreen.opacity(0.85))
                    }
                }
            }
            .padding(.horizontal, 28)
            .padding(.top, 8)

            Spacer()
            Spacer()
        }
        .padding(.horizontal, 16)
    }
}
