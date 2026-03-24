package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;

public class SimpleAlignAndShoot extends Command {
  public final Rotation2d angleDeadband = Rotation2d.fromDegrees(4);
  HoodSubsystem hood;
  TargetingSubsystem targeting;
  ShooterSubsystem shooter;
  MagicFloorSubsystem floor;
  TransporterSubsystem transporter;
  CommandSwerveDrivetrain drivetrain;
  Rotation2d tunableHoodAngle = new Rotation2d();
  double tunablerps = 0;

  private final SwerveRequest.FieldCentricFacingAngle driveRequest =
      new SwerveRequest.FieldCentricFacingAngle();

  public boolean hasSpunUp = false;

  public SimpleAlignAndShoot(
      HoodSubsystem hoodSubsystem,
      TargetingSubsystem targetingSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagicFloorSubsystem magicFloorSubsystem,
      TransporterSubsystem transporterSubsystem,
      CommandSwerveDrivetrain drivetrainSubsystem,
      Rotation2d hoodAngle,
      double rps) {
    hood = hoodSubsystem;
    targeting = targetingSubsystem;
    shooter = shooterSubsystem;
    floor = magicFloorSubsystem;
    transporter = transporterSubsystem;
    drivetrain = drivetrainSubsystem;

    hoodAngle = tunableHoodAngle;
    tunablerps = rps;

    addRequirements(hood, targeting, shooter, floor, transporter, drivetrain);
  }

  @Override
  public void initialize() {
    driveRequest.HeadingController.setPID(6.0, 0.0, 0.5);
    driveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {

    Rotation2d desiredDrivetrainHeading = targeting.getIdealRobotHeading().get();

    if (isFacingHub()) {
      //   shooter.setTargetRPS(targeting.getTargetFlywheelRPS);
      //   hood.setTargetAngle(targeting.getIdealHoodAngle());
      shooter.setTargetRPS(tunablerps);
      hood.setTargetAngle(() -> tunableHoodAngle);

      if ((shooter.isAtSetpoint() || hasSpunUp) && hood.isAtSetpoint()) {
        hasSpunUp = true;
        floor.setVoltageFunction(8);
        transporter.setVoltageFunction(-8);
        // CommandScheduler.getInstance().schedule(floor.setVoltage(8), transporter.setVoltage(-8));
      }

    } else {
      drivetrain.setControl(driveRequest.withTargetDirection(desiredDrivetrainHeading));
    }
  }

  public boolean isFacingHub() {
    return Math.abs(
            drivetrain.getHeading().getRotations()
                - targeting.getIdealRobotHeading().get().getRotations())
        < angleDeadband.getRotations();
  }
}
