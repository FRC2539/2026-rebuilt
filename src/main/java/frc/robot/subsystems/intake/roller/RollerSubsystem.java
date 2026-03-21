package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public RollerSubsystem(RollerIO rollerIO) {
    this.rollerIO = rollerIO;

    setDefaultCommand(setVoltage(0));
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/IntakeSubsystemRoller", inputs);
  }

  public Command setVoltage(double voltage) {
    return Commands.run(
        () -> {
          rollerIO.setVoltage(voltage);
        });
  }

  public Command reverseVoltage(double voltage) {
    return Commands.run(
        () -> {
          rollerIO.setVoltage(-voltage);
        });
  }
}
