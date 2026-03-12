package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.intake.IntakeConstants;

public class RollerIOTalonFX implements RollerIO {
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.ROLLER_MOTOR_CANBUS);

    public RollerIOTalonFX() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration().withCurrentLimits(IntakeConstants.ROLLER_CURRENT_LIMIT);
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(RollerIOInputs inputs) {
        inputs.wheelsVoltage = getRollerVoltage();
    }

    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    public double getRollerVelocity() { // How fast the rollers are currently moving
        return rollerMotor.getVelocity().refresh().getValueAsDouble();
    } 
    public boolean isRollerStationary() { // Simple detection of the roller moving within tolerance
        return Math.abs(getRollerVelocity()) < IntakeConstants.ROLLER_VELOCITY_TOLERANCE;
    } 
    public boolean isRollerMovingForward() { // Simple detection of the roller moving forward within tolerance
        return getRollerVelocity() > IntakeConstants.ROLLER_VELOCITY_TOLERANCE;
    } 
    public boolean isRollerMovingBackward() { // Simple detection of the roller moving backward within tolerance
        return getRollerVelocity() < -IntakeConstants.ROLLER_VELOCITY_TOLERANCE;
    } 
    public double getRollerVoltage() { // Simple voltage reader
        return rollerMotor.getMotorVoltage().refresh().getValueAsDouble();
    } 
}
