package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

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
    return Commands.run(
        () -> {
          pivotIO.setVoltage(voltage);
        });
  }

  public Command setPosition(Rotation2d targetPosition) {
    return runOnce(() -> pivotIO.setPosition(targetPosition))
        .andThen(Commands.run(() -> {}, this));
        //.until(() -> pivotIO.isAtSetpoint());
  }

  public boolean isEncoderConnected() {
    return inputs.throughboreConnected;
  }

  public boolean isDown() {
    if (!isEncoderConnected()) return false;
    double current = inputs.pivotPosition;
    double target = PivotConstants.intakeDownPosition.getRotations();
    return Math.abs(current - target) < PivotConstants.pivotDeadband.getRotations();
  }

  // public Command Feather() {
  //   // TODO: Crunch command for pivot
  //   return Commands.sequence(
  //     PullUp(),
  //     Commands.waitSeconds(0.5),
  //     PutDown(),
  //     Commands.waitSeconds(0.5)
  //   );
  // }

  public Command Crunch() {
    return Commands.either(
        Commands.sequence(
            setPosition(PivotConstants.intakeFeatherPosition),
            Commands.waitSeconds(0.5),
            PutDown()),
        Commands.none(),
        this::isDown);
  }

  public Command Toggle() {
    return Commands.either(PullUp(), PutDown(), this::isDown);
  }
}
