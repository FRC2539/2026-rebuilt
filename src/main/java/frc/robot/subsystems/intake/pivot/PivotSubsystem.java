package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public PivotSubsystem(PivotIO io) {
    pivotIO = io;
    setDefaultCommand(PullUp());
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/Intake", inputs);
  }

  public Command PullUp() {
    return Commands.run(
        () -> {
          pivotIO.setPivotPosition(IntakeConstants.PIVOT_POSITION_UP);
        },
        this);
  }

  public Command PutDown() {
    return Commands.run(
        () -> {
          pivotIO.setPivotPosition(IntakeConstants.PIVOT_POSITION_UP);
        },
        this);
  }

  // In theory...
  public Command Crunch() {
    return Commands.repeatingSequence(
        Commands.deadline(
            Commands.waitSeconds(0.5),
            Commands.run(
                () -> {
                  pivotIO.setPivotPosition(IntakeConstants.PIVOT_POSITION_CRUNCH);
                })),
        Commands.deadline(
            Commands.waitSeconds(1.5),
            Commands.run(
                () -> {
                  pivotIO.setPivotPosition(IntakeConstants.PIVOT_POSITION_UP);
                })));
  }
}
