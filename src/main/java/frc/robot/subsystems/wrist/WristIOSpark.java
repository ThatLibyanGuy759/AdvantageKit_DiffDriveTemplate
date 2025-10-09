package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs;

public class WristIOSpark implements WristIO {
  private final SparkMax wristMotor;
  private final AbsoluteEncoder wristEncoder;
  private final SparkClosedLoopController controller;

  public WristIOSpark() {
    wristMotor = new SparkMax(wristMotorCANId, MotorType.kBrushless);

    wristMotor.configure(
        Configs.WristConfigs.wristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    wristEncoder = wristMotor.getAbsoluteEncoder();
    controller = wristMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    // Assuming the AbsoluteEncoder returns radians (REV 2024+ API)
    inputs.angleRads = wristEncoder.getPosition();
    inputs.velocityRadPerSec = wristEncoder.getVelocity();
    inputs.appliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.currentAmps = wristMotor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    wristMotor.setVoltage(volts);
  }

  @Override
  public void setPercent(double percent) {
    wristMotor.set(percent);
  }

  @Override
  public void setPositionRadians(double positionRads) {
    controller.setReference(positionRads, ControlType.kPosition);
  }

  @Override
  public void stop() {
    wristMotor.stopMotor();
  }
}