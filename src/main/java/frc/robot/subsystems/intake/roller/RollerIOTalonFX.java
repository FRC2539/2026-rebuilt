package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX leftRollerMotor =
      new TalonFX(RollerConstants.leftRollerMotorID, RollerConstants.rollerMotorCanBus);

  private final TalonFX rightRollerMotor =
      new TalonFX(RollerConstants.rightRollerMotorID, RollerConstants.rollerMotorCanBus);

  public RollerIOTalonFX() {

    leftRollerMotor.getConfigurator().apply(RollerConstants.motorConfig);
    rightRollerMotor.getConfigurator().apply(RollerConstants.motorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
  }

  public void updateInputs(RollerIOInputs inputs) {
    inputs.wheelsVoltage = leftRollerMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    leftRollerMotor.setVoltage(voltage);
    rightRollerMotor.setVoltage(voltage);
  }
}
