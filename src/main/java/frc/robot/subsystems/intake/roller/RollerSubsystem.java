package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public RollerSubsystem(RollerIO io) {
    System.out.println("Initializing");
    rollerIO = io;
    setDefaultCommand(Stop());
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/Intake", inputs);
  }

  public Command Stop() {
    return Commands.run(
        () -> {
          rollerIO.setRollerVoltage(IntakeConstants.ROLLER_VOLTAGE_STOP);
        },
        this);
  }

  public Command RunForward() {
    return Commands.run(
        () -> {
          rollerIO.setRollerVoltage(IntakeConstants.ROLLER_VOLTAGE_FORWARD);
        },
        this);
  }

  public Command RunBackward() {
    return Commands.run(
        () -> {
          rollerIO.setRollerVoltage(IntakeConstants.ROLLER_VOLTAGE_BACKWARD);
        },
        this);
  }

  public Command Crunch() {
    // TODO: Crunch command for roller
    return Commands.run(
        () -> {
          rollerIO.setRollerVoltage(IntakeConstants.ROLLER_VOLTAGE_FORWARD);
        },
        this);
  }
}
