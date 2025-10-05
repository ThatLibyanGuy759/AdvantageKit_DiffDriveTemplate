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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs;

/**
 * Elevator IO implementation for REV SparkMax motor controllers.
 * This implementation drives two SparkMax motors - one as leader, one as follower.
 */
public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax elevatorMotorLeft;
  private final SparkMax elevatorMotorRight;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  public ElevatorIOSpark() {
    elevatorMotorLeft = new SparkMax(elevatorMotorLeftCanId, MotorType.kBrushless);
    elevatorMotorRight = new SparkMax(elevatorMotorRightCanId, MotorType.kBrushless);

    // Configure motors
    elevatorMotorLeft.configure(
        Configs.ElevatorConfigs.elevatorFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotorRight.configure(
        Configs.ElevatorConfigs.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set up encoder and controller from the leader motor (right motor)
    encoder = elevatorMotorRight.getEncoder();
    controller = elevatorMotorRight.getClosedLoopController();

    // Reset encoder position
    encoder.setPosition(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // The encoder is already configured to return meters directly
    inputs.positionMeters = encoder.getPosition();
    
    // The encoder velocity is already configured to return meters per second
    inputs.velocityMetersPerSec = encoder.getVelocity();
    
    // Motor voltage and current
    inputs.appliedVolts = elevatorMotorRight.getAppliedOutput() * elevatorMotorRight.getBusVoltage();
    inputs.currentAmps = elevatorMotorRight.getOutputCurrent();
    
    // Limit switches (if implemented in hardware)
    inputs.atUpperLimit = false; // TODO: Implement if you have limit switches
    inputs.atLowerLimit = false; // TODO: Implement if you have limit switches
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMotorRight.setVoltage(volts);
  }

  @Override
  public void setPosition(double positionMeters) {
    // The encoder is configured to use meters directly, so no conversion needed
    controller.setReference(positionMeters, ControlType.kPosition);
  }

  @Override
  public void stop() {
    elevatorMotorRight.stopMotor();
  }

  @Override
  public void resetPosition() {
    encoder.setPosition(0);
  }


}