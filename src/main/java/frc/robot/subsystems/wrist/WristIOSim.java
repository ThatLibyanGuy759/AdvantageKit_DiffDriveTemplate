package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
  // Simple single-jointed arm simulation
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          wristGearReduction,
          wristMOI,
          wristArmLengthMeters,
          minAngleRads,
          maxAngleRads,
          true, // simulate gravity
          initialAngleRads);

  private final PIDController pid = new PIDController(simKp, 0.0, simKd);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double positionSetpoint = initialAngleRads;

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (closedLoop) {
      double pidOut = pid.calculate(armSim.getAngleRads(), positionSetpoint);
      appliedVolts = MathUtil.clamp(pidOut, -12.0, 12.0);
    }

    armSim.setInputVoltage(appliedVolts);
    armSim.update(0.02);

    inputs.angleRads = armSim.getAngleRads();
    inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = armSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPercent(double percent) {
    setVoltage(MathUtil.clamp(percent, -1.0, 1.0) * 12.0);
  }

  @Override
  public void setPositionRadians(double positionRads) {
    closedLoop = true;
    positionSetpoint = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
    pid.setSetpoint(positionSetpoint);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
