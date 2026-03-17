package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {

  private double targetRPS = 0;
  VelocityVoltage controlRequest = new VelocityVoltage(targetRPS);

  TalonFX leadMotor = new TalonFX(ShooterConstants.leadMotorID);
  TalonFX rightLowerMotor = new TalonFX(ShooterConstants.rightLowerMotorID);
  TalonFX leftUpperMotor = new TalonFX(ShooterConstants.leftUpperMotorID);
  TalonFX leftLowerMotor = new TalonFX(ShooterConstants.leftLowerMotorID);

  Follower opposedFollowRequest =
      new Follower(ShooterConstants.leadMotorID, MotorAlignmentValue.Opposed);

  public ShooterIOTalonFX() {
    leadMotor.getConfigurator().apply(ShooterConstants.talonFXConfig);
    rightLowerMotor.getConfigurator().apply(ShooterConstants.talonFXConfig);
    leftUpperMotor.getConfigurator().apply(ShooterConstants.talonFXConfig);
    leftLowerMotor.getConfigurator().apply(ShooterConstants.talonFXConfig);

    rightLowerMotor.setControl(
        new Follower(ShooterConstants.leadMotorID, MotorAlignmentValue.Aligned));
    leftLowerMotor.setControl(opposedFollowRequest);
    leftUpperMotor.setControl(opposedFollowRequest);

    leadMotor.setVoltage(0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.currentRPS = leadMotor.getRotorVelocity().getValueAsDouble();
    inputs.voltage = leadMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    leadMotor.setVoltage(voltage);
  }

  @Override
  public void setControlVelocityRPS(double targetVelocityRPS) {
    this.targetRPS = targetVelocityRPS;

    leadMotor.setControl(controlRequest.withVelocity(targetVelocityRPS));

    rightLowerMotor.setControl(
        new Follower(ShooterConstants.leadMotorID, MotorAlignmentValue.Aligned));
    leftLowerMotor.setControl(opposedFollowRequest);
    leftUpperMotor.setControl(opposedFollowRequest);
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(leadMotor.getVelocity().getValueAsDouble() - targetRPS)
        < ShooterConstants.goalDeadbandRPS;
  }
}
