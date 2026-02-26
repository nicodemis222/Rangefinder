//
//  RangefinderApp.swift
//  Rangefinder
//
//  Entry point. Gates tutorial on first launch.
//  Camera session starts only after tutorial is dismissed.
//

import SwiftUI

@main
struct RangefinderApp: App {
    @StateObject private var appState = AppState()
    @AppStorage("hasCompletedTutorial") private var hasCompletedTutorial = false
    @AppStorage("tutorialCategory") private var tutorialCategory = ""

    /// Resolve stored category string to enum (nil = show category picker)
    private var initialCategory: TutorialCategory? {
        TutorialCategory(rawValue: tutorialCategory)
    }

    var body: some Scene {
        WindowGroup {
            ZStack {
                RangefinderView()
                    .environmentObject(appState)

                if !hasCompletedTutorial {
                    TutorialView(
                        onComplete: {
                            withAnimation(.easeInOut(duration: 0.4)) {
                                hasCompletedTutorial = true
                                tutorialCategory = ""
                            }
                        },
                        initialCategory: initialCategory
                    )
                    .transition(.opacity)
                    .zIndex(100)
                }
            }
            .preferredColorScheme(.dark)
            .task(id: hasCompletedTutorial) {
                if hasCompletedTutorial {
                    await appState.startSession()
                }
            }
        }
    }
}
