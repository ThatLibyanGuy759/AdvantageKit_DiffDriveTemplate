package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  // Preserve original API behavior

  /** Rotate wrist to a position specified in degrees (closed-loop). */
  public void wristRotateToPosition(double positionDegrees) {
    io.setPositionRadians(Rotation2d.fromDegrees(positionDegrees).getRadians());
  }

  /** Return wrist angle in degrees. */
  public double getWristAngle() {
    return Rotation2d.fromRadians(inputs.angleRads).getDegrees();
  }

  /**
   * Print/log the wrist position in degrees (replaces SmartDashboard with AdvantageKit logging).
   */
  public void printWristPosition() {
    Logger.recordOutput("Wrist/AngleDegrees", getWristAngle());
  }

  /** Set wrist open-loop speed (-1 to 1), matching old motor.set(). */
  public void setWristSpeed(double speed) {
    io.setPercent(speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }
}
