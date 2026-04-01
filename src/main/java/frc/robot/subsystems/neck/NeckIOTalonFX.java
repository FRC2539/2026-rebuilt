package frc.robot.subsystems.neck;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.transporter.TransporterConstants;
import frc.robot.subsystems.transporter.TransporterIO.TransporterIOInputs;

public class NeckIOTalonFX implements NeckIO{

    TalonFX motor = new TalonFX(NeckConstants.motorID);
    private double targetRPS = 0;
    VelocityVoltage controlRequest = new VelocityVoltage(targetRPS);

  public NeckIOTalonFX() {

    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(45))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));

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
