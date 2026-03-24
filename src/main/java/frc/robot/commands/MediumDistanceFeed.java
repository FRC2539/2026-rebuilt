package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.controller.Axis;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;
import java.util.function.Supplier;

public class MediumDistanceFeed extends Command {

  private final Command command;

  private static final Rotation2d RED_WALL = Rotation2d.fromDegrees(0);
  private static final Rotation2d BLUE_WALL = Rotation2d.fromDegrees(180);

  private static final double BASE_RPS = 55.0;
  private static final Rotation2d BASE_HOOD = Rotation2d.fromRotations(0.06);

  public MediumDistanceFeed(
      CommandSwerveDrivetrain drivetrain,
      TargetingSubsystem targeting,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      TransporterSubsystem transporter,
      MagicFloorSubsystem magicFloor,
      Axis xAxis,
      Axis yAxis,
      Supplier<Double> shooterOffset,
      Supplier<Double> hoodOffset) {

    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();

    request.HeadingController.setPID(6.0, 0.0, 0.1);
    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    Command driveAndFace =
        Commands.run(
            () -> {
              double xInput = -Math.pow(xAxis.get(), 3);
              double yInput = -Math.pow(yAxis.get(), 3);

              double maxSpeed = DriveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond);

              Rotation2d heading = drivetrain.getHeading();

              double xSpeed = (xInput * heading.getCos() - yInput * heading.getSin()) * maxSpeed;

              double ySpeed = (xInput * heading.getSin() + yInput * heading.getCos()) * maxSpeed;

              Rotation2d targetWall =
                  (DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == Alliance.Blue)
                      ? BLUE_WALL
                      : RED_WALL;

              drivetrain.setControl(
                  request
                      .withVelocityX(xSpeed)
                      .withVelocityY(ySpeed)
                      .withTargetDirection(targetWall));
            },
            drivetrain);

    Command spinUp =
        Commands.run(
            () -> {
              double rps = BASE_RPS + shooterOffset.get();

              Rotation2d hoodAngle = BASE_HOOD.plus(Rotation2d.fromRotations(hoodOffset.get()));

              shooter.setTargetRPS(rps);
              hood.setTargetAngle(() -> hoodAngle);
            },
            shooter,
            hood);

    Command shoot =
        Commands.run(
            () -> {
              if (shooter.isAtSetpoint() && hood.isAtSetpoint()) {
                transporter.setVoltage(-8);
                magicFloor.setVoltage(8);
              } else {
                transporter.setVoltage(0);
                magicFloor.setVoltage(0);
              }
            },
            transporter,
            magicFloor);

    command = Commands.parallel(driveAndFace, spinUp, shoot);

    addRequirements(drivetrain, shooter, hood, transporter, magicFloor);
  }
}
