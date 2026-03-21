package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  
  public void updateInputs(PivotIOInputs pivotIO);

  @AutoLog
  public class PivotIOInputs {
    double pivotVoltage = 0;
    double pivotPosition = 0;
  }

  public void setPosition(double position);

  public void setVoltage(double position);

  public boolean isAtSetpoint(); // Simple detection of the pivot being within range within tolerance  
}
