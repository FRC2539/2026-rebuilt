package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.geometry.Rotation2d;

public final class HoodConstants { // .0654

  public static final int kMotorId = 10;
  public static final int hoodEncoderID = 41;
  public static final String kCanBus = "rio";

  public static final Rotation2d maxHoodAngle = Rotation2d.fromRotations(0.255615); //0.2255// THIS IS THE FERRYING ANGLE // .1301 //.8427 // . 00864
  public static final Rotation2d minHoodAngle = Rotation2d.fromRotations(-0.016);//-0.1506 // -.2284// .33502 // .4665 -.315186 // .4665

  public static final Rotation2d angleDeadband = Rotation2d.fromRotations(0.008);
  public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  public static final ExternalFeedbackConfigs feedbackConfig =
      new ExternalFeedbackConfigs()
          .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.RemoteCANcoder)
          .withFeedbackRemoteSensorID(hoodEncoderID)
          .withSensorToMechanismRatio(1);

  public static final Slot0Configs slot0configs = new Slot0Configs().withKP(38).withKS(0).withKV(0);

  public static final MotorOutputConfigs outputConfigs =
      new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);

  public static final TalonFXSConfiguration hoodMotorConfig =
      new TalonFXSConfiguration()
          .withExternalFeedback(feedbackConfig)
          .withSlot0(slot0configs)
          .withMotorOutput(outputConfigs)
          .withCommutation(
              new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST));
}
