package frc.robot.subsystems.magicFloor;

import org.littletonrobotics.junction.AutoLog;

public interface MagicFloorIO {

    public void updateInputs(MagicFloorIOInputs inputs);

    @AutoLog
    public class MagicFloorIOInputs {
        double voltage = 0;
        double speed = 0;
    }

    public void setVoltage(double voltage);
}
