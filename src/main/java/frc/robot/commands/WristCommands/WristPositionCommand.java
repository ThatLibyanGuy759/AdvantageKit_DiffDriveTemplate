// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

/** Wrist position command using AdvantageKit-style Wrist subsystem. */
public class WristPositionCommand extends Command {
  private final Wrist wrist;
  private final double targetPosition;

  public WristPositionCommand(Wrist wrist) {
    this.wrist = wrist;
    this.targetPosition = 0.0;
    addRequirements(wrist);
  }

  public WristPositionCommand(Wrist wrist, double positionDegrees) {

    this.wrist = wrist;
    this.targetPosition = positionDegrees;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    wrist.wristRotateToPosition(targetPosition);
  }

  @Override
  public void execute() {
    wrist.printWristPosition();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double angle = wrist.getWristAngle();
    return (targetPosition - 2 < angle && targetPosition + 2 > angle);
  }
}
