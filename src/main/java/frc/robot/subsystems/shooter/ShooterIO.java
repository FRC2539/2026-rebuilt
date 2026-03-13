package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface ShooterIO {

  public void updateInputs(ShooterIOInputs inputs);

  public void setVoltage(double voltage);

  @AutoLog
  public class ShooterIOInputs {
    double currentRPS = 0;
    double voltage = 0;
  }

  @AutoLogOutput
  public boolean isAtSetpoint();

  @AutoLogOutput
  public void setControlVelocityRPS(double targetVelocity);

  public double getExpectedDelta();
}
