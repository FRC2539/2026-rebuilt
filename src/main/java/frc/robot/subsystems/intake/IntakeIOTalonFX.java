package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, IntakeConstants.PIVOT_MOTOR_CANBUS);
  private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.ROLLER_MOTOR_CANBUS);

    public void updateInputs(IntakeIOInputs inputs) {
        // TODO: Intake: Implement updateInputs()
    }

    public void setRollerVoltage(double voltage) {
        // TODO: Intake: Implement setRollerVoltage()
    }
    public void setPivotPosition(double position) {
        // TODO: Intake: Implement setPivotPosition()
    }

    public boolean isAtSetpoint() { // Simple detection of the pivot being within range within tolerance
        return true; // TODO: Intake: Implement isAtSetpoint()
    } 
    public double getPivotDelta() { // How much the pivot needs to turn to be at its absolute target
        return 0; // TODO: Intake: Implement getPivotDelta()
    }
    public double getPivotVelocity() { // How fast the pivot is currently moving
        return 0; // TODO: Intake: Implement getPivotVelocity()
    } 

    public boolean isRollerStationary() { // Simple detection of the roller moving within tolerance
        return true; // TODO: Intake: Implement isRollerStationary()
    } 
    public double getRollerVelocity() { // How fast the rollers are currently moving 
        return 0; // TODO: Intake: Implement getRollerVelocity()
    } 
}