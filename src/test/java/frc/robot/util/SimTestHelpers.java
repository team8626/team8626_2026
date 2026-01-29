// Copyright 2025-2026 FRC 8626
// https://github.com/team8626
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Helper utilities for simulation integration tests. Provides methods for testing PID convergence,
 * oscillation detection, and steady-state error validation.
 */
public class SimTestHelpers {
  /** Default simulation timestep in seconds (50 Hz) */
  public static final double DEFAULT_TIMESTEP = 0.02;

  /** Result of a convergence test */
  public static class ConvergenceResult {
    public final boolean converged;
    public final double finalError;
    public final double timeToConverge;
    public final int oscillationCount;
    public final List<Double> errorHistory;

    public ConvergenceResult(
        boolean converged,
        double finalError,
        double timeToConverge,
        int oscillationCount,
        List<Double> errorHistory) {
      this.converged = converged;
      this.finalError = finalError;
      this.timeToConverge = timeToConverge;
      this.oscillationCount = oscillationCount;
      this.errorHistory = errorHistory;
    }
  }

  /**
   * Simulate and test convergence of a system toward a setpoint.
   *
   * @param getMeasurement Supplier that returns the current measured value
   * @param setpoint The target setpoint value
   * @param tolerance The acceptable error tolerance for "converged"
   * @param maxDuration Maximum simulation time in seconds
   * @param updateFunction Runnable to call each timestep to advance the simulation
   * @return ConvergenceResult with details about the convergence behavior
   */
  public static ConvergenceResult testConvergence(
      DoubleSupplier getMeasurement,
      double setpoint,
      double tolerance,
      double maxDuration,
      Runnable updateFunction) {
    return testConvergence(
        getMeasurement, setpoint, tolerance, maxDuration, DEFAULT_TIMESTEP, updateFunction);
  }

  /**
   * Simulate and test convergence of a system toward a setpoint.
   *
   * @param getMeasurement Supplier that returns the current measured value
   * @param setpoint The target setpoint value
   * @param tolerance The acceptable error tolerance for "converged"
   * @param maxDuration Maximum simulation time in seconds
   * @param timestep Simulation timestep in seconds
   * @param updateFunction Runnable to call each timestep to advance the simulation
   * @return ConvergenceResult with details about the convergence behavior
   */
  public static ConvergenceResult testConvergence(
      DoubleSupplier getMeasurement,
      double setpoint,
      double tolerance,
      double maxDuration,
      double timestep,
      Runnable updateFunction) {

    List<Double> errorHistory = new ArrayList<>();
    double elapsedTime = 0.0;
    double convergeTime = -1.0;
    int oscillationCount = 0;
    Double lastError = null;
    Boolean lastErrorPositive = null;

    while (elapsedTime < maxDuration) {
      // Run one simulation step
      updateFunction.run();

      // Calculate error
      double measurement = getMeasurement.getAsDouble();
      double error = setpoint - measurement;
      errorHistory.add(error);

      // Detect oscillation (zero crossing of error)
      boolean errorPositive = error >= 0;
      if (lastErrorPositive != null && errorPositive != lastErrorPositive) {
        oscillationCount++;
      }
      lastErrorPositive = errorPositive;
      lastError = error;

      // Check convergence
      if (Math.abs(error) <= tolerance && convergeTime < 0) {
        convergeTime = elapsedTime;
      }

      elapsedTime += timestep;
    }

    boolean converged = Math.abs(lastError) <= tolerance;
    return new ConvergenceResult(
        converged,
        lastError != null ? lastError : Double.NaN,
        convergeTime >= 0 ? convergeTime : maxDuration,
        oscillationCount,
        errorHistory);
  }

  /**
   * Assert that a system converges to a setpoint within specified parameters.
   *
   * @param getMeasurement Supplier that returns the current measured value
   * @param setpoint The target setpoint value
   * @param tolerance The acceptable error tolerance
   * @param maxTimeSeconds Maximum allowed time to converge
   * @param maxOscillations Maximum allowed zero-crossings (oscillations)
   * @param updateFunction Runnable to advance the simulation
   */
  public static void assertConvergesWithin(
      DoubleSupplier getMeasurement,
      double setpoint,
      double tolerance,
      double maxTimeSeconds,
      int maxOscillations,
      Runnable updateFunction) {

    ConvergenceResult result =
        testConvergence(getMeasurement, setpoint, tolerance, maxTimeSeconds, updateFunction);

    assertTrue(
        result.converged,
        String.format(
            "System did not converge. Final error: %.4f, tolerance: %.4f",
            result.finalError, tolerance));

    assertTrue(
        result.timeToConverge <= maxTimeSeconds,
        String.format(
            "Convergence too slow. Time: %.3fs, max allowed: %.3fs",
            result.timeToConverge, maxTimeSeconds));

    assertTrue(
        result.oscillationCount <= maxOscillations,
        String.format(
            "Too many oscillations. Count: %d, max allowed: %d",
            result.oscillationCount, maxOscillations));
  }

  /**
   * Assert that the steady-state error is within tolerance after settling.
   *
   * @param getMeasurement Supplier that returns the current measured value
   * @param setpoint The target setpoint value
   * @param tolerance The acceptable steady-state error tolerance
   * @param settlingTime Time to wait before measuring steady-state (seconds)
   * @param measurementDuration Duration to measure steady-state (seconds)
   * @param updateFunction Runnable to advance the simulation
   */
  public static void assertSteadyStateError(
      DoubleSupplier getMeasurement,
      double setpoint,
      double tolerance,
      double settlingTime,
      double measurementDuration,
      Runnable updateFunction) {

    // Wait for settling
    int settlingSteps = (int) (settlingTime / DEFAULT_TIMESTEP);
    for (int i = 0; i < settlingSteps; i++) {
      updateFunction.run();
    }

    // Measure steady-state error
    int measurementSteps = (int) (measurementDuration / DEFAULT_TIMESTEP);
    double maxError = 0.0;
    double totalError = 0.0;

    for (int i = 0; i < measurementSteps; i++) {
      updateFunction.run();
      double error = Math.abs(setpoint - getMeasurement.getAsDouble());
      maxError = Math.max(maxError, error);
      totalError += error;
    }

    double avgError = totalError / measurementSteps;

    assertTrue(
        maxError <= tolerance,
        String.format(
            "Steady-state error too large. Max: %.4f, tolerance: %.4f", maxError, tolerance));

    assertTrue(
        avgError <= tolerance,
        String.format(
            "Average steady-state error too large. Avg: %.4f, tolerance: %.4f",
            avgError, tolerance));
  }

  /**
   * Detect if a system is oscillating excessively.
   *
   * @param getMeasurement Supplier that returns the current measured value
   * @param setpoint The target setpoint value
   * @param testDuration Duration to test for oscillation (seconds)
   * @param updateFunction Runnable to advance the simulation
   * @return Number of zero-crossings detected (oscillations)
   */
  public static int detectOscillations(
      DoubleSupplier getMeasurement,
      double setpoint,
      double testDuration,
      Runnable updateFunction) {

    ConvergenceResult result =
        testConvergence(getMeasurement, setpoint, Double.MAX_VALUE, testDuration, updateFunction);

    return result.oscillationCount;
  }

  /**
   * Calculate rise time (time to go from 10% to 90% of setpoint).
   *
   * @param errorHistory List of errors over time
   * @param initialError Initial error at t=0
   * @param timestep Time between samples
   * @return Rise time in seconds, or -1 if not reached
   */
  public static double calculateRiseTime(
      List<Double> errorHistory, double initialError, double timestep) {
    if (errorHistory.isEmpty() || initialError == 0) {
      return -1;
    }

    double tenPercentError = initialError * 0.9;
    double ninetyPercentError = initialError * 0.1;

    double tenPercentTime = -1;
    double ninetyPercentTime = -1;

    for (int i = 0; i < errorHistory.size(); i++) {
      double error = Math.abs(errorHistory.get(i));
      double time = i * timestep;

      if (tenPercentTime < 0 && error <= Math.abs(tenPercentError)) {
        tenPercentTime = time;
      }
      if (ninetyPercentTime < 0 && error <= Math.abs(ninetyPercentError)) {
        ninetyPercentTime = time;
        break;
      }
    }

    if (tenPercentTime >= 0 && ninetyPercentTime >= 0) {
      return ninetyPercentTime - tenPercentTime;
    }
    return -1;
  }

  /**
   * Calculate overshoot percentage.
   *
   * @param errorHistory List of errors over time
   * @param initialError Initial error at t=0
   * @return Overshoot as a percentage (e.g., 10.0 for 10% overshoot), or 0 if no overshoot
   */
  public static double calculateOvershoot(List<Double> errorHistory, double initialError) {
    if (errorHistory.isEmpty() || initialError == 0) {
      return 0;
    }

    boolean initialPositive = initialError > 0;
    double maxOvershoot = 0;

    for (Double error : errorHistory) {
      // Overshoot occurs when error crosses zero and goes negative (if initially positive)
      boolean isOvershoot = (initialPositive && error < 0) || (!initialPositive && error > 0);

      if (isOvershoot) {
        double overshootAmount = Math.abs(error);
        maxOvershoot = Math.max(maxOvershoot, overshootAmount);
      }
    }

    return (maxOvershoot / Math.abs(initialError)) * 100.0;
  }

  /**
   * Run simulation for a specified duration without checking convergence.
   *
   * @param durationSeconds Duration to run
   * @param updateFunction Runnable to advance the simulation
   */
  public static void runSimulation(double durationSeconds, Runnable updateFunction) {
    int steps = (int) (durationSeconds / DEFAULT_TIMESTEP);
    for (int i = 0; i < steps; i++) {
      updateFunction.run();
    }
  }
}
