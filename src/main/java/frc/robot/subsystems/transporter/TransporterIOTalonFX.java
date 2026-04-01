package frc.robot.subsystems.transporter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TransporterIOTalonFX implements TransporterIO {

  TalonFX motor = new TalonFX(TransporterConstants.motorID);

  public TransporterIOTalonFX() {

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
  public void updateInputs(TransporterIOInputs inputs) {
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.speed = motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double transportVoltage) {
    motor.setVoltage(transportVoltage);
  }
}
