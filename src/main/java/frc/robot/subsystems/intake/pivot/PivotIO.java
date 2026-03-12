package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    public void updateInputs(PivotIOInputs pivotIO);

    @AutoLog
    public class PivotIOInputs {
      double pivotVoltage = 0;
      double pivotPosition = 0;
    }

    public void setPivotPosition(double position);

    public void setPID(double kp, double ki, double kd);

    public double getPivotPosition(); // Absolute position of the pivot
    public double getPivotDelta(); // How much the pivot needs to turn to be at its absolute target
    public boolean isAtSetpoint(); // Simple detection of the pivot being within range within tolerance
    public double getPivotVelocity(); // How fast the pivot is currently moving
    public double getPivotVoltage();
}
