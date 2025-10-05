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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem using AdvantageKit IO pattern. This subsystem can work with real hardware or
 * simulation depending on the IO implementation provided.
 */
public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Elevator preset positions (in meters)
  public static final double GROUND_POSITION = 0.0;
  public static final double LOW_POSITION = 0.3;
  public static final double MID_POSITION = 0.8;
  public static final double HIGH_POSITION = 1.2;
  public static final double GROUND_POSITION_OPPORATOR = 0.15;
  public static final double LOW_POSITION_OPPORATOR = 0.5;
  public static final double MID_POSITION_OPPORATOR = 1;
  public static final double HIGH_POSITION_OPPORATOR = 1.4;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  /** Directly command the elevator in open loop voltage. */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Directly command the elevator to a closed-loop position. */
  public void setPosition(double positionMeters) {
    io.setPosition(positionMeters);
  }

  /** Immediately stop all elevator motion. */
  public void stopMotion() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Returns the current elevator height in meters. */
  @AutoLogOutput(key = "Elevator/HeightMeters")
  public double getHeight() {
    return inputs.positionMeters;
  }

  /** Returns the current elevator velocity in meters per second. */
  @AutoLogOutput(key = "Elevator/VelocityMetersPerSec")
  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  /** Returns true if the elevator is at the specified position within tolerance. */
  public boolean atPosition(double targetMeters, double toleranceMeters) {
    return Math.abs(inputs.positionMeters - targetMeters) <= toleranceMeters;
  }

  /** Returns true if the elevator is at the specified position within default tolerance (2cm). */
  public boolean atPosition(double targetMeters) {
    return atPosition(targetMeters, 0.02); // 2cm tolerance
  }

  /** Command to run the elevator at a specified voltage. */
  public Command runVoltage(double volts) {
    return runEnd(() -> io.setVoltage(volts), () -> io.stop());
  }

  /** Command to run the elevator at a percentage of max voltage. */
  public Command runPercent(double percent) {
    return runVoltage(percent * 12.0);
  }

  /** Command to move elevator to a specific position and hold it there. */
  public Command moveToPosition(double positionMeters) {
    return runEnd(() -> io.setPosition(positionMeters), () -> io.stop());
  }

  /** Command to move elevator to ground position. */
  public Command moveToGround() {
    return moveToPosition(GROUND_POSITION);
  }

  /** Command to move elevator to low position. */
  public Command moveToLow() {
    return moveToPosition(LOW_POSITION);
  }

  /** Command to move elevator to mid position. */
  public Command moveToMid() {
    return moveToPosition(MID_POSITION);
  }

  /** Command to move elevator to high position. */
  public Command moveToHigh() {
    return moveToPosition(HIGH_POSITION);
  }

  public Command moveToGroundOpporator() {
    return moveToPosition(GROUND_POSITION_OPPORATOR);
  }

  /** Command to move elevator to low position. */
  public Command moveToLowOpporator() {
    return moveToPosition(LOW_POSITION_OPPORATOR);
  }

  /** Command to move elevator to mid position. */
  public Command moveToMidOpporator() {
    return moveToPosition(MID_POSITION_OPPORATOR);
  }

  /** Command to move elevator to high position. */
  public Command moveToHighOpporator() {
    return moveToPosition(HIGH_POSITION_OPPORATOR);
  }

  /** Command for manual elevator control using joystick/triggers. */
  public Command runTeleop(DoubleSupplier upInput, DoubleSupplier downInput) {
    return runEnd(
        () -> {
          double input = upInput.getAsDouble() - downInput.getAsDouble();
          io.setVoltage(input * 12.0);
        },
        () -> io.stop());
  }

  /** Command to stop the elevator. */
  public Command stop() {
    return runOnce(() -> io.stop());
  }

  /**
   * Command to reset the elevator encoder position to zero. Use this when the elevator is at the
   * bottom position.
   */
  public Command resetPosition() {
    return runOnce(() -> io.resetPosition());
  }

  /** Returns true if the elevator is at the upper limit. */
  @AutoLogOutput(key = "Elevator/AtUpperLimit")
  public boolean atUpperLimit() {
    return inputs.atUpperLimit;
  }

  /** Returns true if the elevator is at the lower limit. */
  @AutoLogOutput(key = "Elevator/AtLowerLimit")
  public boolean atLowerLimit() {
    return inputs.atLowerLimit;
  }
}
