package frc.robot.subsystems.transporter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.transporter.TransporterIO.TransporterIOInputs;

public class TransporterSubsystem extends SubsystemBase {

  TransporterIO transporterIO;
  TransporterIOInputs inputs = new TransporterIOInputs();

  public TransporterSubsystem(TransporterIO transporterIO) {
    this.transporterIO = transporterIO;

    setDefaultCommand(setVoltage(0));
  }

  @Override
  public void periodic() {
    transporterIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/TransporterSubsystem", inputs);
  }

  public Command setVoltage(double voltage){
    return Commands.run(
      () -> transporterIO.setVoltage(voltage)
    );
  }
  
  public Command reverseVoltage(double voltage){
    return Commands.run(
      () -> transporterIO.setVoltage(-voltage)
    );
  }
}
