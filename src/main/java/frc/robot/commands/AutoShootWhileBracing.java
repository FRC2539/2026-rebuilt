package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;
import java.util.function.Supplier;

public class AutoShootWhileBracing extends Command {

  private final Command command;
  private static final double ANGLE_TOLERANCE_RAD = 0.05;

  public AutoShootWhileBracing(
      CommandSwerveDrivetrain drivetrain,
      TargetingSubsystem targeting,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      TransporterSubsystem transporter,
      MagicFloorSubsystem magicFloor,
      Supplier<Double> shooterOffset,
      Supplier<Double> hoodOffset) {

    SwerveRequest.FieldCentricFacingAngle aimRequest = new SwerveRequest.FieldCentricFacingAngle();

    aimRequest.HeadingController.setPID(6.0, 0.0, 0.1);
    aimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    Command rotateToAngle =
        Commands.run(
                () -> {
                  Rotation2d targetAngle = targeting.getIdealRobotHeading().get();

                  drivetrain.setControl(
                      aimRequest
                          .withVelocityX(0)
                          .withVelocityY(0)
                          .withTargetDirection(targetAngle));
                },
                drivetrain)
            .until(
                () -> {
                  Rotation2d current = drivetrain.getHeading();
                  Rotation2d target = targeting.getIdealRobotHeading().get();

                  double error = MathUtil.angleModulus(target.minus(current).getRadians());

                  return Math.abs(error) < ANGLE_TOLERANCE_RAD;
                });

    Command spinUp =
        Commands.run(
            () -> {
              double targetRPS = targeting.getIdealFlywheelRPS().get() + shooterOffset.get();

              Rotation2d targetHood =
                  targeting
                      .getIdealHoodAngle()
                      .get()
                      .plus(Rotation2d.fromRotations(hoodOffset.get()));

              shooter.setTargetRPS(targetRPS);
              hood.setTargetAngle(() -> targetHood);
            },
            shooter,
            hood);

    Command shoot =
        Commands.run(
            () -> {
              transporter.setVoltage(-8);
              magicFloor.setVoltage(8);
            },
            transporter,
            magicFloor);

    command =
        Commands.sequence(
            Commands.parallel(rotateToAngle, spinUp),
            Commands.waitUntil(() -> shooter.isAtSetpoint() && hood.isAtSetpoint()),
            shoot);
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
    return command.isFinished();
  }
}
