package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class RobotContainer {

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final Auto auto;

  public RobotContainer() {
    configureBindings();

    auto = new Auto(this);
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return auto.getAutoCommand();
  }
}