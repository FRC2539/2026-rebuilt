package frc.robot.subsystems.neck;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.neck.NeckIO.NeckIOInputs;
import java.util.function.Supplier;

public class NeckSubsystem extends SubsystemBase {

  NeckIO neckIO;
  NeckIOInputs inputs = new NeckIOInputs();

  public NeckSubsystem(NeckIO neckIO) {
    this.neckIO = neckIO;

    setDefaultCommand(setVoltage(0));
  }

  @Override
  public void periodic() {
    neckIO.updateInputs(inputs);
    // Logger.processInputs("RealOutputs/NeckSubsystem", inputs);
  }

  public void setTargetRPS(double targetRPS) {
    neckIO.setControlVelocityRPS(targetRPS);
  }

  public Command setNeckRPSCommand(Supplier<Double> desiredRPS) {
    return Commands.runOnce(() -> this.setTargetRPS(desiredRPS.get()), this)
        .andThen(Commands.run(() -> {}, this))
        .until(this::isAtSetpoint);
  }

  public Command setNeckRPSForever(double desiredRPS) {
    return Commands.runOnce(() -> this.setTargetRPS(desiredRPS), this)
        .andThen(Commands.run(() -> {}, this));
  }

  public Command setVoltage(double voltage) {
    return run(() -> neckIO.setVoltage(voltage));
  }

  public boolean isAtSetpoint() {
    return neckIO.isAtSetpoint();
  }
}
