package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  public void updateInputs(RollerIOInputs rollerIO);

  @AutoLog
  public class RollerIOInputs {
    double wheelsVoltage = 0;
  }

  public void setRollerVoltage(double voltage);

  public double getRollerVelocity(); // How fast the rollers are currently moving

  public boolean isRollerStationary(); // Simple detection of the roller moving within tolerance

  public boolean isRollerMovingForward(); // Simple detection of the roller moving within tolerance

  public boolean isRollerMovingBackward(); // Simple detection of the roller moving within tolerance

  public double getRollerVoltage();
}
