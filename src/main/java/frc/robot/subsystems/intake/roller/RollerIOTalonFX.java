package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.intake.IntakeConstants;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.ROLLER_MOTOR_CANBUS);

  public RollerIOTalonFX() {
    rollerMotor.setVoltage(0);

    TalonFXConfiguration rollerConfig =
        new TalonFXConfiguration().withCurrentLimits(IntakeConstants.ROLLER_CURRENT_LIMIT)
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  public void updateInputs(RollerIOInputs inputs) {
    inputs.wheelsVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }
}
