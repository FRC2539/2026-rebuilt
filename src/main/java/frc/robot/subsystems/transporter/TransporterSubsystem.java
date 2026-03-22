package frc.robot.subsystems.transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TransporterSubsystem extends SubsystemBase {

  TransporterIO transporterIO;
  TransporterIOInputsAutoLogged inputs = new TransporterIOInputsAutoLogged();

  public TransporterSubsystem(TransporterIO transporterIO) {
    this.transporterIO = transporterIO;

    setDefaultCommand(setVoltage(0));
  }

  @Override
  public void periodic() {
    transporterIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/TransporterSubsystem", inputs);
  }

  public Command setVoltage(double voltage) {
    return run(() -> transporterIO.setVoltage(voltage));
  }

  public Command reverseVoltage(double voltage) {
    return run(() -> transporterIO.setVoltage(-voltage));
  }
}
