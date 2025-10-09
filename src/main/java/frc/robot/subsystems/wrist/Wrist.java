package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

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

  private static final double PIVOT_Y_M = 0.0; // from zeroedPosition[1]
  private static final double PIVOT_Z_M = 0.090298; // from zeroedPosition[2]

  /** If your model’s zero orientation doesn’t match code zero, add an offset (degrees). */
  private static final double WRIST_ZERO_OFFSET_DEG = 0.0;

  /** Total number of animated components you want to publish (>= WRIST_COMPONENT_INDEX + 1). */
  private static final int COMPONENT_COUNT = 3; // model_0, model_1, model_2 (wrist)

  // =======================
  // Continuous angle tracking (multi-turn)
  // =======================
  private boolean initialized = false;
  private double lastWrappedRad = 0.0; // last raw (wrapped) angle from IO, in radians
  private double continuousRad = 0.0; // unwrapped, multi-turn angle in radians
  private final Elevator elevator;

  public Wrist(WristIO io, Elevator elevator) {
    this.io = io;
    this.elevator = elevator;
  }

  /** Closed-loop to a position specified in DEGREES (ABSOLUTE multi-turn). */
  public void wristRotateToAbsoluteDegrees(double absoluteDegrees) {
    double targetRad = Math.toRadians(absoluteDegrees);
    // Place the target near our current continuous angle so it follows the intended turn count
    double placed = placeInScope(targetRad, continuousRad);
    io.setPositionRadians(placed); // IO should NOT wrap; use multi-turn position control
  }

  /**
   * Legacy API: rotate to position in degrees (kept for compatibility). NOTE: This may take the
   * shortest path in a single-turn domain depending on your IO. Prefer
   * wristRotateToAbsoluteDegrees(...) for multi-turn moves like -270.
   */
  public void wristRotateToPosition(double positionDegrees) {
    io.setPositionRadians(Rotation2d.fromDegrees(positionDegrees).getRadians());
  }

  /** Return wrist angle in degrees (wrapped -180..180). */
  public double getWristAngle() {
    return Rotation2d.fromRadians(inputs.angleRads).getDegrees();
  }

  /** Set wrist open-loop speed (-1 to 1). */
  public void setWristSpeed(double speed) {
    io.setPercent(speed);
  }

  public void printWristPosition() {
    Logger.recordOutput("Wrist/AngleDegrees", getWristAngle());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    double wrapped = inputs.angleRads;
    if (!initialized) {
      initialized = true;
      lastWrappedRad = wrapped;
      continuousRad = wrapped;
    } else {
      double delta = MathUtil.inputModulus(wrapped - lastWrappedRad, -Math.PI, Math.PI);
      continuousRad += delta;
      lastWrappedRad = wrapped;
    }

    Logger.recordOutput("Wrist/AngleWrappedDeg", Math.toDegrees(wrapped));
    Logger.recordOutput("Wrist/AngleContinuousDeg", Math.toDegrees(continuousRad));

    Pose3d robotPose = new Pose3d(0.03, 0.09, 0.9, new Rotation3d(0.0, 0.0, 0.0));

    double elevatorHeight = elevator.getElevatorHeightMeters();

    double visRad = continuousRad + Math.toRadians(WRIST_ZERO_OFFSET_DEG);
    Rotation3d wristRot = new Rotation3d(0.0, visRad, 0.0);

    Transform3d wristXform =
        new Transform3d(PIVOT_X_M, PIVOT_Y_M, PIVOT_Z_M + elevatorHeight, wristRot);

    Pose3d wristPose = robotPose.plus(wristXform);

    Logger.recordOutput("Wrist/ComponentPose", wristPose);
    Logger.recordOutput("Wrist/AngleDegrees", getWristAngle());
  }

  // ----------------- Helpers -----------------

  /**
   * Place target angle (rad) in the same "scope" as reference (rad), preserving multi-turn intent.
   */
  private static double placeInScope(double targetRad, double referenceRad) {
    while (targetRad - referenceRad > Math.PI) targetRad -= 2.0 * Math.PI;
    while (targetRad - referenceRad <= -Math.PI) targetRad += 2.0 * Math.PI;
    return targetRad;
  }
}
