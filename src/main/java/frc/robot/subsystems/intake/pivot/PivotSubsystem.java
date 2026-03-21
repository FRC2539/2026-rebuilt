package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public PivotSubsystem(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
    setDefaultCommand(PullUp());
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/IntakeSubsystemPivot", inputs);
  }

  public Command PullUp() {
    return Commands.run(
        () -> {
          pivotIO.setPosition(IntakeConstants.PIVOT_POSITION_UP);
        });
  }

  public Command PutDown() {
    return Commands.run(
        () -> {
          pivotIO.setPosition(IntakeConstants.PIVOT_POSITION_DOWN);
        });
  }

  public Command setVoltage(double voltage){
    return Commands.run(
      () -> {
        pivotIO.setVoltage(voltage);
      });
  }

  public Command Crunch() {
    // TODO: Crunch command for pivot
    return Commands.run(
        () -> {
          pivotIO.setPosition(IntakeConstants.PIVOT_POSITION_CRUNCH);
        });
  }
}
