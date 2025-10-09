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
import com.revrobotics.RelativeEncoder;

public class WristIOSpark implements WristIO {
  private final SparkMax wristMotor;
  private final AbsoluteEncoder absEnc;
  private final RelativeEncoder relEnc;
  private final SparkClosedLoopController controller;
  private boolean seeded = false;

  public WristIOSpark() {
    wristMotor = new SparkMax(wristMotorCANId, MotorType.kBrushless);
    wristMotor.configure(
        Configs.WristConfigs.wristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Primary relative encoder (multi-turn)
    relEnc = wristMotor.getEncoder();
    // Absolute encoder for boot alignment only
    absEnc = wristMotor.getAbsoluteEncoder();

    controller = wristMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    // Seed the relative encoder ONCE from absolute (place in current scope ~ just copy at boot)
    if (!seeded) {
      relEnc.setPosition(absEnc.getPosition()); // both are in radians due to config
      seeded = true;
    }

    inputs.angleRads = relEnc.getPosition();            // multi-turn feedback
    inputs.velocityRadPerSec = relEnc.getVelocity();
    inputs.appliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.currentAmps = wristMotor.getOutputCurrent();
  }

  @Override public void setVoltage(double volts) { wristMotor.setVoltage(volts); }
  @Override public void setPercent(double percent) { wristMotor.set(percent); }

  @Override
  public void setPositionRadians(double positionRads) {
    // Multi-turn reference on primary encoder (no PID wrapping)
    controller.setReference(positionRads, ControlType.kPosition);
  }

  @Override public void stop() { wristMotor.stopMotor(); }
}
