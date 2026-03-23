package frc.robot.subsystems.magicFloor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class MagicFloorConstants {

  public static int magicMotorID = 17;
  //VOLTAGE IS POSTIVE LIKE 6

  public static final String magicCanBus = "rio";

  public static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(40);

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration().withCurrentLimits(currentLimits);
}
