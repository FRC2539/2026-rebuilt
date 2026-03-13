package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, IntakeConstants.PIVOT_MOTOR_CANBUS);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.ROLLER_MOTOR_CANBUS);
    
    private PIDController pivotController = new PIDController(0.6, 0, 0);

    public IntakeIOTalonFX() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration().withCurrentLimits(IntakeConstants.PIVOT_CURRENT_LIMIT);
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration().withCurrentLimits(IntakeConstants.ROLLER_CURRENT_LIMIT);
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotPosition = getPivotPosition();
        inputs.pivotVoltage = getPivotVoltage();
        inputs.wheelsVoltage = getRollerVoltage();
    }

    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }
    public void setPivotPosition(double position) {
        pivotController.setSetpoint(position);
    }

    public void setPID(double kp, double ki, double kd) {
        pivotController.setPID(kp, ki, kd);
    }

    public double getPivotPosition() { // Absolute position of the pivot
        return pivotMotor.getMotorVoltage().refresh().getValueAsDouble();
    }
    public double getPivotDelta() { // How much the pivot needs to turn to be at its absolute target
        return pivotController.getSetpoint() - getPivotPosition();
    }
    public boolean isAtSetpoint() { // Simple detection of the pivot being within range within tolerance
        return Math.abs(getPivotDelta()) <= IntakeConstants.PIVOT_ROTATION_TOLERANCE;
    } 
    public double getPivotVelocity() { // How fast the pivot is currently moving
        return pivotMotor.getVelocity().refresh().getValueAsDouble();
    } 
    public double getPivotVoltage() {
        return pivotMotor.getMotorVoltage().refresh().getValueAsDouble();
    } 

    public double getRollerVelocity() { // How fast the rollers are currently moving
        return rollerMotor.getVelocity().refresh().getValueAsDouble();
    } 
    public boolean isRollerStationary() { // Simple detection of the roller moving within tolerance
        return Math.abs(getRollerVelocity()) < IntakeConstants.ROLLER_VELOCITY_TOLERANCE;
    } 
    public double getRollerVoltage() {
        return rollerMotor.getMotorVoltage().refresh().getValueAsDouble();
    } 
}