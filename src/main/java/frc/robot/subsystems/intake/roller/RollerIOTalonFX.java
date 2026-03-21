package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.hardware.TalonFX;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX rollerMotor =
      new TalonFX(RollerConstants.rollerMotorID, RollerConstants.rollerMotorCanBus);

  public RollerIOTalonFX() {

    rollerMotor.getConfigurator().apply(RollerConstants.motorConfig);
  }

  public void updateInputs(RollerIOInputs inputs) {
    inputs.wheelsVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }
}
