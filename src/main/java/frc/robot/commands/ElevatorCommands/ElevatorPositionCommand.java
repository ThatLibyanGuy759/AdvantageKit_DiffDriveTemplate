package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.elevator.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to move the elevator to a specific position using AdvantageKit pattern. */
public class ElevatorPositionCommand extends Command {
  private final Elevator elevator;
  private final double targetPosition;
  private final double tolerance;

  /**
   * Creates a new ElevatorPositionCommand to move to ground position.
   *
   * @param elevator The elevator subsystem.
   */
  public ElevatorPositionCommand(Elevator elevator) {
    this.elevator = elevator;
    this.targetPosition = 0.0;
    this.tolerance = 0.02; // 2cm tolerance
    addRequirements(elevator);
  }

  /**
   * Creates a new ElevatorPositionCommand to move to a specific position.
   *
   * @param elevator The elevator subsystem.
   * @param position The target position in meters.
   */
  public ElevatorPositionCommand(Elevator elevator, double position) {
    this.elevator = elevator;
    this.targetPosition = position;
    this.tolerance = 0.02; // 2cm tolerance
    addRequirements(elevator);
  }

  /**
   * Creates a new ElevatorPositionCommand with custom tolerance.
   *
   * @param elevator The elevator subsystem.
   * @param position The target position in meters.
   * @param tolerance The position tolerance in meters.
   */
  public ElevatorPositionCommand(Elevator elevator, double position, double tolerance) {
    this.elevator = elevator;
    this.targetPosition = position;
    this.tolerance = tolerance;
    addRequirements(elevator);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start moving to target position
    elevator.moveToPosition(targetPosition).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The IO layer and periodic() handle the position control
    // No additional work needed here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the elevator when command ends
    elevator.stop().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atPosition(targetPosition, tolerance);
  }
}