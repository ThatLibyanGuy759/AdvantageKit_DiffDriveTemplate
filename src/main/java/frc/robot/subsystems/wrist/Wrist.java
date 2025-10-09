package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private static final double PIVOT_Y_M = 0.0;      // from zeroedPosition[1]
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
  private double continuousRad = 0.0;  // unwrapped, multi-turn angle in radians

  public Wrist(WristIO io) {
    this.io = io;
  }

  /** Closed-loop to a position specified in DEGREES (ABSOLUTE multi-turn). */
  public void wristRotateToAbsoluteDegrees(double absoluteDegrees) {
    double targetRad = Math.toRadians(absoluteDegrees);
    // Place the target near our current continuous angle so it follows the intended turn count
    double placed = placeInScope(targetRad, continuousRad);
    io.setPositionRadians(placed); // IO should NOT wrap; use multi-turn position control
  }

  /** Legacy API: rotate to position in degrees (kept for compatibility).
   *  NOTE: This may take the shortest path in a single-turn domain depending on your IO.
   *  Prefer wristRotateToAbsoluteDegrees(...) for multi-turn moves like -270.
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

    // === 1) Build continuous (multi-turn) angle from wrapped sensor ===
    double wrapped = inputs.angleRads; // typically -π..π from IO
    if (!initialized) {
      initialized = true;
      lastWrappedRad = wrapped;
      continuousRad = wrapped; // seed
    } else {
      // Unwrap across the discontinuity using inputModulus in (-π, π]
      double delta = MathUtil.inputModulus(wrapped - lastWrappedRad, -Math.PI, Math.PI);
      continuousRad += delta;
      lastWrappedRad = wrapped;
    }

    // Debug logs (help verify unwrapping)
    Logger.recordOutput("Wrist/AngleWrappedDeg", Math.toDegrees(wrapped));
    Logger.recordOutput("Wrist/AngleContinuousDeg", Math.toDegrees(continuousRad));

    // === 2) Compose like Elevator (do NOT publish Robot/Pose here) ===
    // If another subsystem already publishes Robot/Pose, do not log it here.
    // Re-create the fixed pose you used (matching your current file):
    Pose3d robotPose =
        new Pose3d(0.03, 0.09, 0.8, new Rotation3d(0.0, 0.0, Math.toRadians(0.0)));

    // Visualization angle = continuous + zero offset
    double visRad = continuousRad + Math.toRadians(WRIST_ZERO_OFFSET_DEG);

    // Choose the correct axis for your wrist rotation:
    //   Y-axis (pitch) shown here; if wrong, try Z (yaw) or X (roll) instead.
    Rotation3d wristRot = new Rotation3d(0.0, visRad, 0.0);
    // For Z axis: Rotation3d wristRot = new Rotation3d(0.0, 0.0, visRad);
    // For X axis: Rotation3d wristRot = new Rotation3d(visRad, 0.0, 0.0);

    Transform3d wristXform = new Transform3d(PIVOT_X_M, PIVOT_Y_M, PIVOT_Z_M, wristRot);
    Pose3d wristPose = robotPose.plus(wristXform);

    // === 3) Publish a SINGLE Pose3d for this mechanism (same pattern as Elevator) ===
    Logger.recordOutput("Wrist/ComponentPose", wristPose);

    // Optional scalar for sanity (wrapped degrees)
    Logger.recordOutput("Wrist/AngleDegrees", getWristAngle());
  }

  // ----------------- Helpers -----------------

  /** Place target angle (rad) in the same "scope" as reference (rad), preserving multi-turn intent. */
  private static double placeInScope(double targetRad, double referenceRad) {
    while (targetRad - referenceRad > Math.PI)  targetRad -= 2.0 * Math.PI;
    while (targetRad - referenceRad <= -Math.PI) targetRad += 2.0 * Math.PI;
    return targetRad;
  }
}
