package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOTalonFX implements PivotIO {

  private final CANcoder pivotEncoder =
      new CANcoder(PivotConstants.pivotEncoderID, PivotConstants.pivotEncoderCanBus);
  private final TalonFX pivotMotor =
      new TalonFX(PivotConstants.pivotMotorID, PivotConstants.pivotMotorCanBus);

  double positionSetpoint = 0;
  PositionVoltage magicVoltage = new PositionVoltage(positionSetpoint);

  public PivotIOTalonFX() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    pivotEncoder.getConfigurator().apply(encoderConfig);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.getConfigurator().apply(PivotConstants.motorConfig);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotPosition = pivotMotor.getPosition().getValueAsDouble();
    inputs.throughboreConnected = pivotEncoder.isConnected();
  }

  public void setPosition(Rotation2d position) {
    positionSetpoint = position.getRotations();
    pivotMotor.setControl(magicVoltage.withPosition(positionSetpoint));
  }

  public boolean isAtSetpoint() {
    return Math.abs(pivotMotor.getPosition().getValueAsDouble() - positionSetpoint)
        < PivotConstants.pivotDeadband.getRotations();
  }

  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }
}
