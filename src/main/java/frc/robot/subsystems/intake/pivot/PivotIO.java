package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  public void updateInputs(PivotIOInputs pivotIO);

  @AutoLog
  public class PivotIOInputs {
    double pivotPosition = 0;
  }

  public void setPosition(Rotation2d position);

  public void setVoltage(double position);

  public boolean isAtSetpoint();
}
