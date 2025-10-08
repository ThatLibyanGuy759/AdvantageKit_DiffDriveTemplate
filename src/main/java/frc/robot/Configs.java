package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

public final class Configs {

  public static final class ElevatorConfigs {
    public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    static {
      elevatorConfig.closedLoopRampRate(0.1).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
      elevatorConfig
          .encoder
          .positionConversionFactor(
              ElevatorConstants.discCircumferenceMeter / ElevatorConstants.discGearRatio)
          .velocityConversionFactor(
              (ElevatorConstants.discCircumferenceMeter / ElevatorConstants.discGearRatio) / 60.0);
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(ElevatorConstants.kP)
          .i(ElevatorConstants.kI)
          .d(ElevatorConstants.kD)
          .outputRange(ElevatorConstants.minOutput, ElevatorConstants.maxOutput);
      elevatorFollowerConfig
          .apply(elevatorConfig)
          .follow(ElevatorConstants.elevatorMotorRightCanId, true);
    }
  }

  // Wrist config without SparkMaxAbsoluteEncoderConfig
  public static final class WristConfigs {
    public static final SparkMaxConfig wristConfig = new SparkMaxConfig();

    static {
      wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(WristConstants.smartCurrentLimit);

      wristConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .p(WristConstants.kP)
          .i(WristConstants.kI)
          .d(WristConstants.kD)
          .outputRange(WristConstants.minOutput, WristConstants.maxOutput);
    }
  }

  // TODO: Add other configs if needed
}
