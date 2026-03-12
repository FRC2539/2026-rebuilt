package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  public void updateInputs(IntakeIOInputs rollerIO);

  @AutoLog
  public class IntakeIOInputs {
    double pivotVoltage = 0;
    double pivotPosition = 0;
    double wheelsVoltage = 0;
  }

  public void setRollerVoltage(double voltage);
  public void setPivotPosition(double position);

  public boolean isAtSetpoint(); // Simple detection of the pivot being within range within tolerance
  public double getPivotDelta(); // How much the pivot needs to turn to be at its absolute target
  public double getPivotVelocity(); // How fast the pivot is currently moving

  public boolean isRollerStationary(); // Simple detection of the roller moving within tolerance
  public double getRollerVelocity(); // How fast the rollers are currently moving
}
