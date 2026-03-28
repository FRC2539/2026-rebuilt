package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SimpleAlignAndShoot;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Auto {

  private final LoggedDashboardChooser<Command> autoChooser;
  private final RobotContainer container;

  private final SwerveRequest.ApplyRobotSpeeds autoRequest =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  public Auto(RobotContainer container) {
    this.container = container;

    configureAutoBuilder();
    registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
  }

  private void configureAutoBuilder() {

    CommandSwerveDrivetrain drivetrain = container.drivetrain;

    RobotConfig config = DriveConstants.getRobotConfigPathplanner();

    AutoBuilder.configure(
        drivetrain::getRobotPose,
        drivetrain::resetPose,
        drivetrain::getRobotSpeeds,
        (speeds, feedforwards) ->
            drivetrain.setControl(
                autoRequest
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        new PPHolonomicDriveController(
            new PIDConstants(8.0, 0.0, 0.0), // translation
            new PIDConstants(4.0, 0.0, 0.0) // rotation
            ),
        config,
        () -> {
          Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        drivetrain);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "shoot",
        new SimpleAlignAndShoot(
            container.hood,
            container.targeting,
            container.shooter,
            container.magicFloor,
            container.transporter,
            container.drivetrain,
            new Rotation2d(),
            0, new Rotation2d(), 0));
    NamedCommands.registerCommand("intake-deploy", container.pivot.setVoltage(-5).withTimeout(2.25).andThen(container.pivot.setVoltage(0)));
    NamedCommands.registerCommand("intake", container.roller.setVoltage(12));
  }

  public Command getAutoCommand() {
    return autoChooser.get();
  }
}
