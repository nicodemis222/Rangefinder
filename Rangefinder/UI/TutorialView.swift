//
//  TutorialView.swift
//  Rangefinder
//
//  SYSTEM BRIEFING — tactical onboarding.
//  Terse, technical wording. Military aesthetic.
//

import SwiftUI

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

// MARK: - Tutorial Content

extension TutorialPage {
    static let pages: [TutorialPage] = [
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

// MARK: - Tutorial View

struct TutorialView: View {
    let onComplete: () -> Void
    @State private var currentPage = 0

    var body: some View {
        ZStack {
            // Solid black background
            Color.black
                .ignoresSafeArea()

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

                // Page content
                TabView(selection: $currentPage) {
                    ForEach(Array(TutorialPage.pages.enumerated()), id: \.element.id) { index, page in
                        TutorialCardView(page: page)
                            .tag(index)
                    }
                }
                .tabViewStyle(.page(indexDisplayMode: .always))
                .indexViewStyle(.page(backgroundDisplayMode: .always))

                // Bottom button
                Button(action: {
                    if currentPage < TutorialPage.pages.count - 1 {
                        withAnimation {
                            currentPage += 1
                        }
                    } else {
                        onComplete()
                    }
                }) {
                    Text(currentPage < TutorialPage.pages.count - 1 ? "NEXT" : "ACKNOWLEDGE")
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
