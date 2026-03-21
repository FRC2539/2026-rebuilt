package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;

public class AutoShootWhileBracing extends Command {

  private final Command command;

  private static final double ANGLE_TOLERANCE_RAD = 0.05;

  public AutoShootWhileBracing(
      CommandSwerveDrivetrain drivetrain,
      TargetingSubsystem targeting,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      TransporterSubsystem transporter,
      MagicFloorSubsystem magicFloor) {

    SwerveRequest.FieldCentricFacingAngle aimRequest = new SwerveRequest.FieldCentricFacingAngle();

    aimRequest.HeadingController.setPID(6.0, 0.0, 0.1);
    aimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    Command rotateToAngle =
        Commands.run(
                () ->
                    drivetrain.setControl(
                        aimRequest
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withTargetDirection(targeting.getIdealRobotHeading().get())),
                drivetrain)
            .until(
                () -> {
                  double current = drivetrain.getHeading().getRadians();
                  double target = targeting.getIdealRobotHeading().get().getRadians();

                  double error = MathUtil.angleModulus(target - current);

                  return Math.abs(error) < ANGLE_TOLERANCE_RAD;
                });

    Command lockWheels = Commands.run(() -> drivetrain.setControl(brakeRequest), drivetrain);

    Command spinUp =
        Commands.parallel(
            Commands.run(
                () -> shooter.setTargetRPS(targeting.getIdealFlywheelRPS().get()), shooter),
            Commands.run(() -> hood.setTargetAngle(targeting.getIdealHoodAngle()), hood));

    Command waitUntilReady =
        Commands.waitUntil(() -> shooter.isAtSetpoint() && hood.isAtSetpoint());

    Command shoot = Commands.parallel(transporter.setVoltage(10), magicFloor.setVoltage(10));

    command =
        Commands.sequence(
            rotateToAngle,
            Commands.parallel(lockWheels),
            spinUp,
            waitUntilReady,
            Commands.parallel(lockWheels, spinUp, shoot));
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
