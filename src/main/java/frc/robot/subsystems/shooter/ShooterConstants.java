package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConstants {

  public static int leadMotorID = 15;
  public static int rightLowerMotorID = 14;
  public static int leftUpperMotorID = 13;
  public static int leftLowerMotorID = 12;

  public static final double goalDeadbandRPS = 2;

  public static final Slot0Configs SlotConfigs =
      new Slot0Configs().withKP(0.45).withKI(0).withKD(0).withKS(1.8).withKA(0).withKV(0);

  public static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(45);

  public static TalonFXConfiguration talonFXConfig =
      new TalonFXConfiguration()
          .withSlot0(SlotConfigs)
          .withCurrentLimits(currentLimits)
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
}
