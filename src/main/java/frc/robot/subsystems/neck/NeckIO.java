package frc.robot.subsystems.neck;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface NeckIO {

    public void updateInputs(NeckIOInputs inputs);

    public void setVoltage(double voltage);

    @AutoLog
    public class NeckIOInputs{
        double voltage = 0;
        double speed = 0;
        double currentRPS = 0;
    }

    @AutoLogOutput
    public boolean isAtSetpoint();

    @AutoLogOutput
    public void setControlVelocityRPS(double targetVelocity);
}
