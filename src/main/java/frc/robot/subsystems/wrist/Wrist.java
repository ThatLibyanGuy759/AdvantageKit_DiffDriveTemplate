package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  // =======================
  // AdvantageScope animation config
  // =======================

  /** Index of the wrist component (maps to model_2.glb). */
  private static final int WRIST_COMPONENT_INDEX = 2;

  /** Pivot of the wrist relative to the robot base, in meters (from config.json). */
  private static final double PIVOT_X_M = 0.090238; // from zeroedPosition[0]
  private static final double PIVOT_Y_M = 0.0;      // from zeroedPosition[1]
  private static final double PIVOT_Z_M = 0.090298; // from zeroedPosition[2]

  /** If your model’s zero orientation doesn’t match code zero, add an offset (degrees). */
  private static final double WRIST_ZERO_OFFSET_DEG = 0.0;

  /** Total number of animated components you want to publish (>= WRIST_COMPONENT_INDEX + 1). */
  private static final int COMPONENT_COUNT = 3; // model_0, model_1, model_2 (wrist)

  public Wrist(WristIO io) {
    this.io = io;
  }

  /** Rotate wrist to a position specified in degrees (closed-loop). */
  public void wristRotateToPosition(double positionDegrees) {
    io.setPositionRadians(Rotation2d.fromDegrees(positionDegrees).getRadians());
  }

  /** Return wrist angle in degrees. */
  public double getWristAngle() {
    return Rotation2d.fromRadians(inputs.angleRads).getDegrees();
  }

  /** Log the wrist position in degrees (AdvantageKit). */
  public void printWristPosition() {
    Logger.recordOutput("Wrist/AngleDegrees", getWristAngle());
  }

  /** Set wrist open-loop speed (-1 to 1). */
  public void setWristSpeed(double speed) {
    io.setPercent(speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    // === 1) DO NOT publish Robot/Pose here if another subsystem already does ===
    // Prefer to let your drivetrain/elevator be the single publisher of Robot/Pose.
    // If NOTHING else publishes robot pose, uncomment the next 3 lines to define it here,
    // but make sure only one subsystem logs "Robot/Pose" at a time.
    // Pose3d robotPose =
    //     new Pose3d(0.35, 0.07, 0.85, new Rotation3d(0.0, 0.0, Math.toRadians(0.0)));
    // Logger.recordOutput("Robot/Pose", robotPose);

    // If another subsystem (like Elevator) already publishes Robot/Pose, *read* that pose
    // via your own tracking (recommended), or re-create the same fixed pose your elevator uses:
    Pose3d robotPose =
        new Pose3d(0.35, 0.07, 0.85, new Rotation3d(0.0, 0.0, Math.toRadians(0.0)));

    // === 2) Wrist angle and axis ===
    double angleRad = inputs.angleRads + Math.toRadians(WRIST_ZERO_OFFSET_DEG);

    // Choose the correct axis for your wrist rotation:
    //   Y-axis (pitch) shown here; if wrong, try Z (yaw) or X (roll) below
    Rotation3d wristRot = new Rotation3d(0.0, angleRad, 0.0);
    // For Z axis: Rotation3d wristRot = new Rotation3d(0.0, 0.0, angleRad);
    // For X axis: Rotation3d wristRot = new Rotation3d(angleRad, 0.0, 0.0);

    // === 3) Wrist pivot location relative to robot origin (meters) ===
    // These match your config.json zeroedPosition, but in meters
    Transform3d wristXform = new Transform3d(
        PIVOT_X_M,    // 0.090238
        PIVOT_Y_M,    // 0.0
        PIVOT_Z_M,    // 0.090298
        wristRot
    );

    // === 4) Compose final wrist pose in field space, like Elevator does ===
    Pose3d wristPose = robotPose.plus(wristXform);

    // === 5) Publish a SINGLE Pose3d for this mechanism (same pattern as Elevator) ===
    Logger.recordOutput("Wrist/ComponentPose", wristPose);

    // Optional: easy sanity metric
    Logger.recordOutput("Wrist/AngleDegrees", getWristAngle());
  }
}
