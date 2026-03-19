package frc.robot.subsystems.magicFloor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MagicFloorIOTalonFX implements MagicFloorIO {

  TalonFX magicMotor = new TalonFX(MagicFloorConstants.magicMotorID);

  public MagicFloorIOTalonFX() {
    magicMotor.setVoltage(0);

    TalonFXConfiguration config = new TalonFXConfiguration();

    magicMotor.getConfigurator().apply(config);
    magicMotor.setNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  @Override
  public void updateInputs(MagicFloorIOInputs inputs) {
    inputs.voltage = magicMotor.getMotorVoltage().getValueAsDouble();
    inputs.speed = magicMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    magicMotor.setVoltage(voltage);
  }
}
