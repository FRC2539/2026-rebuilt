package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  public void updateInputs(RollerIOInputs rollerIO);

  @AutoLog
  public class RollerIOInputs {
    double wheelsVoltage = 0;
  }

  public void setVoltage(double voltage);
}
