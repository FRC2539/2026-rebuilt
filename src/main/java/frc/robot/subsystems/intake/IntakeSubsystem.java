package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        intakeIO = io;
    }



    
  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/Intake", inputs);
  }
}
