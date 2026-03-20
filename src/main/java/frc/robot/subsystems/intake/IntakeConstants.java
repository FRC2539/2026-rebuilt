package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public final class IntakeConstants {
  // TODO: Intake: Define PIVOT_MOTOR_ID
  public static final int PIVOT_MOTOR_ID = 0;
  public static final String PIVOT_MOTOR_CANBUS = "rio";
  // TODO: Intake: Define PIVOT_CURRENT_LIMIT
  public static final CurrentLimitsConfigs PIVOT_CURRENT_LIMIT = new CurrentLimitsConfigs();
  // TODO: Intake: Define PIVOT_ROTATION_TOLERANCE
  public static final double PIVOT_ROTATION_TOLERANCE = 0;

  // TODO: Intake: Define ROLLER_MOTOR_ID
  public static final int ROLLER_MOTOR_ID = 1;
  public static final String ROLLER_MOTOR_CANBUS = "rio";
  // TODO: Intake: Define ROLLER_CURRENT_LIMIT
  public static final CurrentLimitsConfigs ROLLER_CURRENT_LIMIT = new CurrentLimitsConfigs();
  // TODO: Intake: Define ROLLER_VELOCITY_TOLERANCE
  public static final double ROLLER_VELOCITY_TOLERANCE = 0;

  // TODO: Intake: Define PIVOT_POSITION_DOWN
  public static final double PIVOT_POSITION_DOWN = 0;
  // TODO: Intake: Define PIVOT_POSITION_UP
  public static final double PIVOT_POSITION_UP = 0;
  // TODO: Intake: Define PIVOT_POSITION_CRUNCH
  public static final double PIVOT_POSITION_CRUNCH = 0;

  // TODO: Intake: Define ROLLER_VOLTAGE_FORWARD
  public static final double ROLLER_VOLTAGE_STOP = 0;
  // TODO: Intake: Define ROLLER_VOLTAGE_FORWARD
  public static final double ROLLER_VOLTAGE_FORWARD = 1;
  // TODO: Intake: Define ROLLER_VOLTAGE_BACKWARD
  public static final double ROLLER_VOLTAGE_BACKWARD = -1;
}
