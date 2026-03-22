package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {
  public static final int pivotMotorID = 18;
  public static final int pivotEncoderID = 40;
  public static final String pivotMotorCanBus = "rio";
  public static final String pivotEncoderCanBus = "CANivore";

  public static final Rotation2d pivotDeadband = Rotation2d.fromDegrees(2);

  public static final Rotation2d intakeUpPosition = new Rotation2d(0.649414);
  public static final Rotation2d intakeDownPosition = new Rotation2d(-0.064209);
  public static final Rotation2d intakeFeatherPosition = new Rotation2d();

  public static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true);

  public static final FeedbackConfigs feedbackConfig =
      new FeedbackConfigs()
          .withFeedbackRemoteSensorID(pivotEncoderID)
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration().withCurrentLimits(currentLimits).withFeedback(feedbackConfig);


}
