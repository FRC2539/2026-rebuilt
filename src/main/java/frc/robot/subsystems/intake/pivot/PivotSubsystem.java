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
          pivotIO.setPivotPosition(IntakeConstants.PIVOT_POSITION_DOWN);
        },
        this);
  }

  public boolean isUpOrCrunch() {
    double pos = pivotIO.getPivotPosition();
    return Math.abs(pos - IntakeConstants.PIVOT_POSITION_UP)
            <= IntakeConstants.PIVOT_ROTATION_TOLERANCE
        || Math.abs(pos - IntakeConstants.PIVOT_POSITION_CRUNCH)
            <= IntakeConstants.PIVOT_ROTATION_TOLERANCE;
  }

  public Command TogglePivot() {
    return Commands.runOnce(
        () -> {
          if (isUpOrCrunch()) {
            pivotIO.setPivotPosition(IntakeConstants.PIVOT_POSITION_DOWN);
          } else {
            pivotIO.setPivotPosition(IntakeConstants.PIVOT_POSITION_UP);
          }
        },
        this);
  }
}
