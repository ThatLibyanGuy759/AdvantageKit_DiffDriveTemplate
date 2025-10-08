// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

/** Wrist position command using AdvantageKit-style Wrist subsystem. */
public class WristPositionCommand extends Command {
  private final Wrist m_wristSubsystem;
  private final double m_position;

  public WristPositionCommand(Wrist wristSubsystem) {
    addRequirements(wristSubsystem);
    m_wristSubsystem = wristSubsystem;
    m_position = 0.0;
  }

  public WristPositionCommand(Wrist wristSubsystem, double positionDegrees) {
    addRequirements(wristSubsystem);
    m_wristSubsystem = wristSubsystem;
    m_position = positionDegrees;
  }

  @Override
  public void initialize() {
    m_wristSubsystem.wristRotateToPosition(m_position);
  }

  @Override
  public void execute() {
    m_wristSubsystem.printWristPosition();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    double angle = m_wristSubsystem.getWristAngle();
    return (m_position - 2 < angle && m_position + 2 > angle);
  }
}
