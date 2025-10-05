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

package frc.robot;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final class ElevatorConstants {
      public static final double discDiameterMeter = Units.inchesToMeters(2.082);
      public static final double discCircumferenceMeter = discDiameterMeter * Math.PI;
      public static final double discGearRatio = 9; 
      public static final int elevatorMotorLeftCanId = 10;
      public static final int elevatorMotorRightCanId = 11;
      
      public static final double kP = 0.0004;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double minOutput = -1;
      public static final double maxOutput = 1;
  }
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
