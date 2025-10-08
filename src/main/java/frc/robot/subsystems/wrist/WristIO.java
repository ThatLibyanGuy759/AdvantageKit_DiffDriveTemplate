// Copyright 2021-2025
// AdvantageKit-style IO interface for the wrist

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double angleRads = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run open loop at the specified percent output (-1 to 1). */
  public default void setPercent(double percent) {}

  /** Run closed loop to the specified position in radians. */
  public default void setPositionRadians(double positionRads) {}

  /** Stop the wrist motor. */
  public default void stop() {}
}
