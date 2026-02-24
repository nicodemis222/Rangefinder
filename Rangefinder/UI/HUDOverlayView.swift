//
//  HUDOverlayView.swift
//  Rangefinder
//
//  Source blend visualization — shows which depth sources are active
//  and their relative contribution to the fused reading.
//  Mil-spec styling: hard edges, phosphor green palette.
//

import SwiftUI

// MARK: - Source Blend View (bar + legend + semantic decision)

struct SourceBlendView: View {
    let sourceWeights: [DepthSource: Float]

    var body: some View {
        VStack(spacing: 4) {
            // Blend bar
            SourceBlendBar(sourceWeights: sourceWeights)
                .frame(height: 4)

            // Legend (only show active sources)
            HStack(spacing: 10) {
                ForEach(activeSources, id: \.0) { source, _ in
                    HStack(spacing: 3) {
                        RoundedRectangle(cornerRadius: 1)
                            .fill(Theme.sourceColor(source))
                            .frame(width: 6, height: 6)
                        Text(source.shortName)
                            .font(.system(size: 10, weight: .medium, design: .monospaced))
                            .foregroundColor(Theme.milGreenDim)
                    }
                }
            }
        }
    }

    private var activeSources: [(DepthSource, Float)] {
        sourceWeights
            .filter { $0.value > 0.01 }
            .sorted { $0.value > $1.value }
    }
}

// MARK: - Semantic Decision Label

/// Shows which depth source the semantic state machine selected as primary.
struct SemanticDecisionLabel: View {
    let decision: SemanticSourceDecision

    var body: some View {
        HStack(spacing: 4) {
            Image(systemName: decisionIcon)
                .font(.system(size: 9, weight: .medium))
                .foregroundColor(decisionColor.opacity(0.8))
            Text(decisionText)
                .font(.system(size: 9, weight: .bold, design: .monospaced))
                .foregroundColor(decisionColor.opacity(0.7))
        }
    }

    private var decisionText: String {
        switch decision {
        case .lidarPrimary: return "LIDAR"
        case .objectPrimary: return "OBJECT"
        case .demPrimary: return "TERRAIN"
        case .neuralPrimary: return "NEURAL"
        case .geometricPrimary: return "GEO"
        case .stadiametric: return "STADIA"
        case .none: return ""
        }
    }

    private var decisionIcon: String {
        switch decision {
        case .lidarPrimary: return "sensor.fill"
        case .objectPrimary: return "viewfinder"
        case .demPrimary: return "mountain.2.fill"
        case .neuralPrimary: return "brain"
        case .geometricPrimary: return "angle"
        case .stadiametric: return "ruler"
        case .none: return "questionmark"
        }
    }

    private var decisionColor: Color {
        switch decision {
        case .lidarPrimary: return Theme.milGreen
        case .objectPrimary: return Color(red: 0.5, green: 0.5, blue: 0.5)
        case .demPrimary: return Color(red: 0.9, green: 0.5, blue: 0.1)
        case .neuralPrimary: return Theme.milAmber
        case .geometricPrimary: return Color(red: 0.0, green: 0.6, blue: 0.7)
        case .stadiametric: return Color(red: 0.7, green: 0.3, blue: 0.9)
        case .none: return Theme.milGreenDim
        }
    }
}

// MARK: - Source Blend Bar

struct SourceBlendBar: View {
    let sourceWeights: [DepthSource: Float]

    var body: some View {
        GeometryReader { geometry in
            HStack(spacing: 1) {
                ForEach(sortedSources, id: \.0) { source, weight in
                    if weight > 0.01 {
                        Rectangle()
                            .fill(Theme.sourceColor(source))
                            .frame(width: geometry.size.width * CGFloat(normalizedWeight(weight)))
                    }
                }
            }
        }
        .clipShape(RoundedRectangle(cornerRadius: 2))
    }

    private var sortedSources: [(DepthSource, Float)] {
        sourceWeights.sorted { $0.value > $1.value }
    }

    private var totalWeight: Float {
        sourceWeights.values.reduce(0, +)
    }

    private func normalizedWeight(_ weight: Float) -> Float {
        guard totalWeight > 0 else { return 0 }
        return weight / totalWeight
    }
}
