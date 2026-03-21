package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.intake.IntakeConstants;

public class PivotIOTalonFX implements PivotIO {

  private final CANcoder pivotEncoder = new CANcoder(IntakeConstants.PivotEncoderID);
  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, IntakeConstants.PIVOT_MOTOR_CANBUS);

  double positionSetpoint = 0;
  PositionDutyCycle magicVoltage = new PositionDutyCycle(positionSetpoint);

  public PivotIOTalonFX() {
    pivotMotor.setVoltage(0);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    pivotEncoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
      .withCurrentLimits(IntakeConstants.PIVOT_CURRENT_LIMIT)
      .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    pivotMotor.getConfigurator().apply(pivotConfig);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotPosition = pivotMotor.getPosition().getValueAsDouble();
    inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setPosition(double position) {

    positionSetpoint = position;
    pivotMotor.setControl(magicVoltage.withPosition(positionSetpoint));
  }

  public boolean isAtSetpoint() { 
    return Math.abs(pivotMotor.getPosition().getValueAsDouble() - positionSetpoint) < IntakeConstants.PIVOT_DEADBAND;
  }

  public void setVoltage(double voltage){
    pivotMotor.setVoltage(voltage);
  }
}
