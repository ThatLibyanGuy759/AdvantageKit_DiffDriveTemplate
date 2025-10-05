package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ElevatorConstants;

public final class Configs {
    
    
    public static final class ElevatorConfigs{
        public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        static{
                elevatorConfig
                    .closedLoopRampRate(0.1)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
                elevatorConfig.encoder
                    .positionConversionFactor(ElevatorConstants.discCircumferenceMeter / ElevatorConstants.discGearRatio)
                    .velocityConversionFactor((ElevatorConstants.discCircumferenceMeter / ElevatorConstants.discGearRatio) / 60.0);
                /*
                 * Configure the closed loop controller. We want to make sure we set the
                 * feedback sensor as the primary encoder.
                 */
                elevatorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control. We don't need to pass a closed loop
                    // slot, as it will default to slot 0.
                    .p(ElevatorConstants.kP)
                    .i(ElevatorConstants.kI)
                    .d(ElevatorConstants.kD)
                    .outputRange(ElevatorConstants.minOutput, ElevatorConstants.maxOutput);
                elevatorFollowerConfig
                    .apply(elevatorConfig)
                    .follow(ElevatorConstants.elevatorMotorRightCanId, true);
                }               
    }


    // TODO: Add climb configuration if needed
    }
