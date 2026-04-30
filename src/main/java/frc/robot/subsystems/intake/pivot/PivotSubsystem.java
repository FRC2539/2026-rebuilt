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
    return Commands.run(() -> pivotIO.setVoltage(voltage), this);
  }

  public Command setPosition(Rotation2d targetPosition) {
    return runOnce(() -> pivotIO.setPosition(targetPosition))
        .andThen(Commands.waitUntil(() -> pivotIO.isAtSetpoint()));
  }

  public boolean isEncoderConnected() {
    return inputs.throughboreConnected;
  }

  public boolean isDown() {
    return inputs.pivotPosition > .48;// .35
  }

  public Command toggleIntake() {
    return Commands.either(this.PullUp(), this.PutDown(), this::isDown);
  }

  public Command feather() { 
    return Commands.either(
        this.setPosition(Rotation2d.fromRotations(.427)), // feather
        this.PutDown(),
        () -> { // is the intake down, a second random number, basically.
          return inputs.pivotPosition
              > .50; // I really just need an arbitrary number here to serve as a "cutoff" between
          // up and down, .35 used above is too low.
        });
  }
}
