# FRC Robot Code

This repo contains the robot code for our FRC team. Follow the steps below to get set up for development and simulation.

## Prerequisites

Install **WPILib 2025**. The installer includes VS Code, Java, and everything you need to build and deploy. Use the official guide: [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).

- **Supported systems**: Windows 10 or 11 (64-bit), macOS 13.3 or newer (Intel or Apple Silicon), or Ubuntu 22.04 / 24.04 (64-bit).
- **On macOS**: Install Xcode Command Line Tools before running the WPILib installer if you don’t have them yet.

## Cloning the project

You can get the project onto your computer using only VS Code (no command-line Git needed):

1. Open **VS Code** (the one from the WPILib installer).
2. Press **Ctrl+Shift+P** (Windows/Linux) or **Cmd+Shift+P** (macOS) to open the Command Palette.
3. Type **Git: Clone** and select it.
4. Paste the repo URL when asked (e.g. `https://github.com/team8626/team8626_2026.git`).
5. Choose a folder on your computer where the project should go.
6. When VS Code asks to open the cloned repo, click **Open** so the project opens in the current window.

If you skip opening at the end, go to **File → Open Folder** and select the cloned folder (the one that contains `build.gradle`).

## Opening the project

If the project is already on your machine (e.g. on a USB drive or from a teammate):

1. Open **VS Code**.
2. Go to **File → Open Folder** and select the repo’s root folder (the folder that contains `build.gradle`).

## Building

From the project root:

- **macOS / Linux**: Run `./gradlew build`
- **Windows**: Run `gradlew.bat build`

In VS Code you can also press **Ctrl+Shift+P** (Windows/Linux) or **Cmd+Shift+P** (macOS), then choose **Build Robot Code**.

The first build can take a few minutes.

## Running tests

From the project root:

- **macOS / Linux**: Run `./gradlew test`
- **Windows**: Run `gradlew.bat test`

You can also use the Command Palette and look for **Run Robot Code Tests** (or the equivalent in your WPILib version).

## Running simulation

Simulation lets you run and drive the robot code on your computer without the real robot.

**Using VS Code (recommended):**

1. Press **Ctrl+Shift+P** (Windows/Linux) or **Cmd+Shift+P** (macOS).
2. Run **Simulate Robot Code** (or **WPILib: Simulate Robot Code**).
3. If you get a dialog, choose the option that enables the **Sim GUI** so the simulation window appears.
4. When the sim is running, **enable the robot** in the Driver Station (or sim Driver Station) so the robot responds to your controls.

**Using the command line:**

- **macOS / Linux**: Run `./gradlew simulateJava`
- **Windows**: Run `gradlew.bat simulateJava`

In this project the sim GUI is **off by default** (so that AdvantageKit log replay works from the command line). If you run `simulateJava` from the command line and **no simulation window appears**, do one of the following:

- Use **VS Code → Simulate Robot Code** and choose the Sim GUI option there, or
- Turn the GUI on by default: in `build.gradle`, find the line `wpi.sim.addGui().defaultEnabled = false` and change it to `true`. Then `./gradlew simulateJava` will show the sim window. This is optional and only for convenience.

Remember to enable the robot in the Driver Station so the robot moves in sim.

## Team number

The team number is in `.wpilib/wpilib_preferences.json` (the `teamNumber` field). Edit that file if your team uses a different number.

## Code layout

- **Robot code**: `src/main/java/frc/robot/` — subsystems, commands, and the main robot logic.
- **Deploy files**: `src/main/deploy/` — e.g. PathPlanner paths and autos that get copied to the robot.

## Formatting

The project uses Spotless. Code is formatted automatically when you build. You don’t need to run anything extra.

## Deploying to the robot

With the roboRIO powered on and connected to your network:

1. Open the Command Palette (**Ctrl+Shift+P** / **Cmd+Shift+P**).
2. Run **Deploy Robot Code**.

For more on deployment, see the [WPILib deployment docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/deploying-robot-code.html).
