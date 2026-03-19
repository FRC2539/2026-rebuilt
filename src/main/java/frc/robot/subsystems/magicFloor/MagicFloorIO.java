package frc.robot.subsystems.magicFloor;

import org.littletonrobotics.junction.AutoLog;

public interface MagicFloorIO {

  public void updateInputs(MagicFloorIOInputs inputs);

  @AutoLog
  public class MagicFloorIOInputs {
    public double voltage = 0;
    public double speed = 0;
  }

  public void setVoltage(double voltage);
}
