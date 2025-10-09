package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

    // ===== Animate the wrist in AdvantageScope (3D components) =====
    // 1) Current angle (rad) + optional zero offset
    double angleRad = inputs.angleRads + Math.toRadians(WRIST_ZERO_OFFSET_DEG);

    // 2) Build Rotation3d around the correct axis:
    //    - If WRIST rotates around Y (pitch): new Rotation3d(0, angleRad, 0)
    //    - If rotates around Z (yaw):        new Rotation3d(0, 0, angleRad)
    //    - If rotates around X (roll):       new Rotation3d(angleRad, 0, 0)
    Rotation3d wristRot = new Rotation3d(0.0, angleRad, 0.0); // <— default: Y-axis

    // 3) The wrist component’s robot-relative translation (pivot point)
    Translation3d wristPivot = new Translation3d(PIVOT_X_M, PIVOT_Y_M, PIVOT_Z_M);

    // 4) Build the Pose3d for the wrist component
    Pose3d wristPose = new Pose3d(wristPivot, wristRot);

    // 5) Publish Pose3d array for all components up to index 2 (wrist is index 2)
    List<Pose3d> poses = new ArrayList<>(COMPONENT_COUNT);
    for (int i = 0; i < COMPONENT_COUNT; i++) {
      poses.add(new Pose3d()); // identity for non-wrist components
    }
    poses.set(WRIST_COMPONENT_INDEX, wristPose);

    // 6) Record output under a consistent path you'll use in AdvantageScope
    //    This is a StructArray<Pose3d>, which AdvantageScope expects for "Component" objects.
    Logger.recordOutput("Robot/ComponentPoses", poses.toArray(new Pose3d[0]));

    // (Optional) Also publish a simple scalar in degrees for quick sanity checks:
    Logger.recordOutput("Wrist/AngleDegrees", getWristAngle());
  }
}
