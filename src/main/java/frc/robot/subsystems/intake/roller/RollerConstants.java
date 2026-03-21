package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class RollerConstants {
  public static final int rollerMotorID = 0;
  public static final String rollerMotorCanBus = "rio";

  public static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(40);

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration().withCurrentLimits(currentLimits);
}
