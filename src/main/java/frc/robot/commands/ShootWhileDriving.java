package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.controller.Axis;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingConstants;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;
import java.util.function.Supplier;

public class ShootWhileDriving extends Command {

  private final Command command;

  private boolean hasSpunUp = false;

  private final Supplier<Double> shooterOffset;
  private final Supplier<Double> hoodOffset;

  public ShootWhileDriving(
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

    this.shooterOffset = shooterOffset;
    this.hoodOffset = hoodOffset;

    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();

    request.HeadingController.setPID(6.0, 0.0, 0.1);
    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    Command driveAndAim =
        Commands.run(
            () -> {
              double xSpeed =
                  xAxis.get()
                      * DriveConstants.MAX_TRANSLATIONAL_SPEED.in(
                          edu.wpi.first.units.Units.MetersPerSecond);

              double ySpeed =
                  yAxis.get()
                      * DriveConstants.MAX_TRANSLATIONAL_SPEED.in(
                          edu.wpi.first.units.Units.MetersPerSecond);

              ChassisSpeeds fieldSpeeds = drivetrain.getFieldSpeeds();

              double distance = targeting.realDistance;

              double lookaheadTime = 0.25;

              var shot = TargetingConstants.hubShotMap.get(distance);
              if (shot != null) {
                lookaheadTime = shot.timeOfFlight();
              }

              double dx = fieldSpeeds.vxMetersPerSecond * lookaheadTime;
              double dy = fieldSpeeds.vyMetersPerSecond * lookaheadTime;

              Rotation2d baseAngle = targeting.getIdealRobotHeading().get();

              double targetX = Math.cos(baseAngle.getRadians());
              double targetY = Math.sin(baseAngle.getRadians());

              double finalX = targetX + dx;
              double finalY = targetY + dy;

              Rotation2d compensatedAngle = new Rotation2d(Math.atan2(finalY, finalX));

              drivetrain.setControl(
                  request
                      .withVelocityX(xSpeed)
                      .withVelocityY(ySpeed)
                      .withTargetDirection(compensatedAngle));
            },
            drivetrain);

    Command spinUp =
        Commands.run(
            () -> {
              shooter.setTargetRPS(targeting.getIdealFlywheelRPS().get() + shooterOffset.get());

              hood.setTargetAngle(
                  () ->
                      targeting
                          .getIdealHoodAngle()
                          .get()
                          .plus(Rotation2d.fromRotations(hoodOffset.get())));

              if (shooter.isAtSetpoint()) {
                hasSpunUp = true;
              }
            },
            shooter,
            hood);

    Command shoot =
        Commands.run(
            () -> {
              boolean ready = hood.isAtSetpoint() && (shooter.isAtSetpoint() || hasSpunUp);

              if (ready) {
                transporter.setVoltage(-8);
                magicFloor.setVoltage(8);
              } else {
                transporter.setVoltage(0);
                magicFloor.setVoltage(0);
              }
            },
            transporter,
            magicFloor);

    command = Commands.parallel(driveAndAim, spinUp, shoot);

    addRequirements(drivetrain, shooter, hood, transporter, magicFloor);
  }

  @Override
  public void initialize() {
    hasSpunUp = false;
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
