package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {
  public static final int pivotMotorID = 18;
  public static final int pivotEncoderID = 40;
  public static final String pivotMotorCanBus = "CANivore";
  public static final String pivotEncoderCanBus = "CANivore";

  public static final Rotation2d pivotDeadband = Rotation2d.fromDegrees(2);

  public static final Rotation2d intakeUpPosition = Rotation2d.fromRotations(.9);
  public static final Rotation2d intakeDownPosition = Rotation2d.fromRotations(0.2268);
  public static final Rotation2d intakeFeatherPosition = Rotation2d.fromRotations(0.4); // 0.95 

  public static final Slot0Configs slot0configs =
      new Slot0Configs().withKP(26).withKD(0.1).withKS(5); // 26, 5

  public static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true);

  public static final FeedbackConfigs feedbackConfig =
      new FeedbackConfigs()
          .withFeedbackRemoteSensorID(pivotEncoderID)
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(currentLimits)
          .withFeedback(feedbackConfig)
          .withSlot0(slot0configs);
}
