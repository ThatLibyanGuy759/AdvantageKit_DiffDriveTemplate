// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Physics simulation implementation for the elevator subsystem. Uses WPILib's ElevatorSim for
 * realistic elevator physics including gravity.
 */
public class ElevatorIOSim implements ElevatorIO {
  // Elevator physical parameters
  private static final double ELEVATOR_MASS_KG = 5.0; // Carriage mass in kg
  private static final double ELEVATOR_DRUM_RADIUS_METERS = discDiameterMeter / 2.0;
  private static final double MIN_HEIGHT_METERS = 0.0;
  private static final double MAX_HEIGHT_METERS = 1.5; // Maximum elevator height

  // Simulation constants
  private static final double SIM_KP = 50.0; // Higher P gain for simulation
  private static final double SIM_KD = 2.5; // Some D gain for stability

  private final ElevatorSim elevatorSim;
  private final PIDController pidController;

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double positionSetpointMeters = 0.0;

  public ElevatorIOSim() {
    double initialPositionMeters = 0.0;

    // ✅ Fix: Two elements, one for position noise and one for velocity noise
    double[] measurementStdDevs = new double[] {0.001, 0.001};

    elevatorSim =
        new ElevatorSim(
            DCMotor.getNEO(2), // Two NEO motors
            discGearRatio, // Gear reduction
            ELEVATOR_MASS_KG, // Carriage mass
            ELEVATOR_DRUM_RADIUS_METERS, // Drum radius
            MIN_HEIGHT_METERS, // Minimum height
            MAX_HEIGHT_METERS, // Maximum height
            true, // Simulate gravity
            initialPositionMeters, // Starting height
            measurementStdDevs);
    // Create PID controller for position control
    pidController = new PIDController(SIM_KP, 0.0, SIM_KD);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Run closed-loop control if enabled
    if (closedLoop) {
      double pidOutput =
          pidController.calculate(elevatorSim.getPositionMeters(), positionSetpointMeters);
      appliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
    }

    // Update simulation with applied voltage
    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(0.02); // 20ms update period

    // Update inputs from simulation
    inputs.positionMeters = elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();

    // Simulate limit switches
    inputs.atLowerLimit = inputs.positionMeters <= MIN_HEIGHT_METERS + 0.01;
    inputs.atUpperLimit = inputs.positionMeters >= MAX_HEIGHT_METERS - 0.01;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPosition(double positionMeters) {
    closedLoop = true;
    positionSetpointMeters = MathUtil.clamp(positionMeters, MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);
    pidController.setSetpoint(positionSetpointMeters);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void resetPosition() {
    // In simulation, we can reset the position
    elevatorSim.setState(0.0, 0.0); // position = 0, velocity = 0
  }
}
