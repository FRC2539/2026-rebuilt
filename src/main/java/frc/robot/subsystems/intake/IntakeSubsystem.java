package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class IntakeSubsystem extends SubsystemBase {
  PivotSubsystem pivotSubsystem;
  RollerSubsystem rollerSubsystem;

  public IntakeSubsystem(PivotIO pivotIO, RollerIO rollerIO) {
    pivotSubsystem = new PivotSubsystem(pivotIO);
    rollerSubsystem = new RollerSubsystem(rollerIO);

    setDefaultCommand(IntakeUpStatic());
  }

  @Override
  public void periodic() {}

  public Command IntakeUpStatic() { // Pull the intake up and stop the rollers
    return Commands.parallel(pivotSubsystem.PullUp(), rollerSubsystem.setVoltage(0))
        .andThen(Commands.idle(this));
  }

  public Command IntakeUpCrunch() { // Pull the intake up while agitating
    return Commands.parallel(pivotSubsystem.PullUp(), rollerSubsystem.setVoltage(0)) //TODO: voltage for crunch?
        .andThen(Commands.idle(this));
  }

  public Command IntakeDownStop() { // Push the intake down, while not intaking
    return Commands.parallel(pivotSubsystem.PutDown(), rollerSubsystem.setVoltage(0))
        .andThen(Commands.idle(this));
  }

  public Command IntakeDownRun() { // Push the intake down and intake
    return Commands.parallel(pivotSubsystem.PutDown(), rollerSubsystem.setVoltage(IntakeConstants.ROLLER_VOLTAGE_FORWARD))
        .andThen(Commands.idle(this));
  }

  public Command IntakeDownExtake() { // Push the intake down and extake
    return Commands.parallel(pivotSubsystem.PutDown(), rollerSubsystem.reverseVoltage(IntakeConstants.ROLLER_VOLTAGE_BACKWARD))
        .andThen(Commands.idle(this));
  }
}
