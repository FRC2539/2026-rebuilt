package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  public ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  ShooterIO shooterIO;

  public ShooterSubsystem(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;

    setDefaultCommand(setVoltage(0));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("RealOutputs/FlywheelSubsystem", shooterInputs);
  }

  public void setTargetRPS(double targetRPS) {
    shooterIO.setControlVelocityRPS(targetRPS);
  }

  public Command setShooterRPSCommand(Supplier<Double> desiredRPS) {
    return Commands.runOnce(() -> this.setTargetRPS(desiredRPS.get()), this)
        .andThen(Commands.run(() -> {}, this))
        .until(this::isAtSetpoint);
  }

  public Command setShooterRPSForever(double desiredRPS) {
    return Commands.runOnce(() -> this.setTargetRPS(desiredRPS), this)
        .andThen(Commands.run(() -> {}, this));
  }

  public Command setVoltage(double voltage) {
    return run(() -> shooterIO.setVoltage(voltage));
  }

  public boolean isAtSetpoint() {
    return shooterIO.isAtSetpoint();
  }
}
