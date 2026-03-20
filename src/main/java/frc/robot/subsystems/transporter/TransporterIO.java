package frc.robot.subsystems.transporter;

import org.littletonrobotics.junction.AutoLog;

public interface TransporterIO {

    public void updateInputs(TransporterIOInputs inputs);

    public void setVoltage(double voltage);

    @AutoLog
    public class TransporterIOInputs{
        public double voltage = 0.0;
        public double speed = 0.0;
    }
}
