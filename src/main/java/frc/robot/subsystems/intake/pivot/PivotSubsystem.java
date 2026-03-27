package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private boolean isUp = false;

  public PivotSubsystem(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/IntakeSubsystemPivot", inputs);
  }

  public Command PullUp() {
    return setPosition(PivotConstants.intakeUpPosition);
  }

  public Command PutDown() {
    return setPosition(PivotConstants.intakeDownPosition);
  }

  public Command setVoltage(double voltage) {
    return Commands.run(() -> pivotIO.setVoltage(voltage), this);
  }

  public Command setPosition(Rotation2d targetPosition) {
    return runOnce(() -> pivotIO.setPosition(targetPosition));
  }

  public boolean isEncoderConnected() {
    return inputs.throughboreConnected;
  }

  public boolean isDown() {
    if (!isEncoderConnected()) return false;
    // double current = inputs.pivotPosition;
    // double target = PivotConstants.intakeDownPosition.getRotations();
    return inputs.pivotPosition < 0;
  }

  public Command Crunch(RollerSubsystem roller) {
    return Commands.runOnce(() -> isUp = !isUp)
        .andThen(
            Commands.either(
                Commands.sequence(
                    PullUp(),
                    Commands.runOnce(() -> roller.runRoller(RollerConstants.intakeVoltage))),
                Commands.sequence(PutDown(), Commands.runOnce(() -> roller.runRoller(0))),
                () -> isUp));
  }

  // public Command Toggle() {
  //   return Commands.runOnce(() -> isUp = !isUp)
  //       .andThen(Commands.either(PullUp(), PutDown(), () -> isUp));
  // }

  public Command toggleIntake() {
    return Commands.either(this.PullUp(), this.PutDown(), this::isDown);
  }
}
