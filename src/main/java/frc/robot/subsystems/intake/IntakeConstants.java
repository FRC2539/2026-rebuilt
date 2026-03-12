package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public final class IntakeConstants {
    public static final int PIVOT_MOTOR_ID = 0; // TODO: Intake: Define PIVOT_MOTOR_ID
    public static final String PIVOT_MOTOR_CANBUS = "rio"; // TODO: Intake: Define PIVOT_MOTOR_ID
    public static final CurrentLimitsConfigs PIVOT_CURRENT_LIMIT = new CurrentLimitsConfigs(); // TODO: Intake: Define PIVOT_CURRENT_LIMIT
    
    public static final int ROLLER_MOTOR_ID = 1; // TODO: Intake: Define ROLLER_MOTOR_ID
    public static final String ROLLER_MOTOR_CANBUS = "rio"; // TODO: Intake: Define ROLLER_MOTOR_ID
    public static final CurrentLimitsConfigs ROLLER_CURRENT_LIMIT = new CurrentLimitsConfigs(); // TODO: Intake: Define ROLLER_CURRENT_LIMIT
}
