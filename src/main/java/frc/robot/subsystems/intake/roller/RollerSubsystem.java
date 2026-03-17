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
    return Commands.runOnce(
            () -> {
              rollerIO.setRollerVoltage(IntakeConstants.ROLLER_VOLTAGE_STOP);
            })
        .until(
            () -> {
              return rollerIO.isRollerStationary();
            });
  }

  public Command RunForward() {
    return Commands.runOnce(
            () -> {
              rollerIO.setRollerVoltage(IntakeConstants.ROLLER_VOLTAGE_FORWARD);
            })
        .until(
            () -> {
              return rollerIO.isRollerMovingForward();
            });
  }

  public Command RunBackward() {
    return Commands.runOnce(
            () -> {
              rollerIO.setRollerVoltage(IntakeConstants.ROLLER_VOLTAGE_BACKWARD);
            })
        .until(
            () -> {
              return rollerIO.isRollerMovingBackward();
            });
  }

  public Command Crunch() {
    return Commands.none();
  }
}
