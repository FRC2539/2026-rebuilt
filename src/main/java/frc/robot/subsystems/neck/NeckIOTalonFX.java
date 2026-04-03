package frc.robot.subsystems.neck;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.shooter.ShooterConstants;

public class NeckIOTalonFX implements NeckIO {

  TalonFX motor = new TalonFX(NeckConstants.motorID);
  private double targetRPS = 0;
  VelocityVoltage controlRequest = new VelocityVoltage(targetRPS);

  public NeckIOTalonFX() {

    motor.getConfigurator().apply(NeckConstants.talonFXConfig);

    motor.setVoltage(0);
  }

  @Override
  public void updateInputs(NeckIOInputs inputs) {
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.speed = motor.getVelocity().getValueAsDouble();
    inputs.currentRPS = motor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double transportVoltage) {
    motor.setVoltage(transportVoltage);
  }

  @Override
  public void setControlVelocityRPS(double targetVelocityRPS) {
    this.targetRPS = targetVelocityRPS;

    motor.setControl(controlRequest.withVelocity(targetVelocityRPS));
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(motor.getVelocity().getValueAsDouble() - targetRPS)
        < ShooterConstants.goalDeadbandRPS;
  }
}
