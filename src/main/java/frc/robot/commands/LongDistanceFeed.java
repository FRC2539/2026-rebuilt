package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;

public class LongDistanceFeed extends Command {

  private final Command command;

  private static final Rotation2d RED_WALL = Rotation2d.fromDegrees(0);
  private static final Rotation2d BLUE_WALL = Rotation2d.fromDegrees(180);

  public LongDistanceFeed(
      CommandSwerveDrivetrain drivetrain,
      TargetingSubsystem targeting,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      TransporterSubsystem transporter,
      MagicFloorSubsystem magicFloor) {

    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();

    request.HeadingController.setPID(6.0, 0.0, 0.1);
    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    Command faceWall =
        Commands.run(
            () -> {
              Rotation2d targetWall =
                  (DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == Alliance.Blue)
                      ? BLUE_WALL
                      : RED_WALL;

              drivetrain.setControl(
                  request.withVelocityX(0).withVelocityY(0).withTargetDirection(targetWall));
            },
            drivetrain);

    Command spinUp =
        Commands.run(
            () -> {
              double rps = targeting.getIdealFlywheelRPS().get();
              Rotation2d hoodAngle = targeting.getIdealHoodAngle().get();

              shooter.setTargetRPS(rps);
              hood.setTargetAngle(() -> hoodAngle);
            },
            shooter,
            hood);

    Command shoot =
        Commands.run(
            () -> {
              if (shooter.isAtSetpoint() && hood.isAtSetpoint()) {
                transporter.setVoltage(10);
                magicFloor.setVoltage(10);
              } else {
                transporter.setVoltage(0);
                magicFloor.setVoltage(0);
              }
            },
            transporter,
            magicFloor);

    command = Commands.parallel(faceWall, spinUp, shoot);

    addRequirements(drivetrain, shooter, hood, transporter, magicFloor);
  }

  @Override
  public void initialize() {
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
