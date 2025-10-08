// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private final SparkMax wristMotor;
  private final AbsoluteEncoder wristEncoder; 
  private SparkClosedLoopController wristPidController;
  public WristSubsystem() {
    wristMotor = new SparkMax(WristConstants.wristMotorCANId, MotorType.kBrushless);

    wristMotor.configure(Configs.WristConfigs.wristConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristEncoder = wristMotor.getAbsoluteEncoder();
    wristPidController = wristMotor.getClosedLoopController();
  }
  
  public void wristRotateToPosition(double position) {
    Rotation2d desiredAngle = Rotation2d.fromDegrees(position);
    wristPidController.setReference(desiredAngle.getRadians(), ControlType.kPosition);
  }

  public double getWristAngle(){
    return Rotation2d.fromRadians(wristEncoder.getPosition()).getDegrees();
  }

  public void printWristPosition() {
    Rotation2d wristEncoderPosition = Rotation2d.fromRadians(wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Position", (wristEncoderPosition.getDegrees()));
  }
  public void setWristSpeed(double speed){
    wristMotor.set(speed);
  }

  @Override
  public void periodic() {
    // printWristPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}