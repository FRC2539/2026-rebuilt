package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class RollerConstants {
  public static final double intakeVoltage = 12;
  public static final int leftRollerMotorID = 9;
  public static final int rightRollerMotorID = 19;

  public static final String rollerMotorCanBus = "rio";

  public static final CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(60);

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration().withCurrentLimits(currentLimits);
}
