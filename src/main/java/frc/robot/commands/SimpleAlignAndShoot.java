package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;

public class SimpleAlignAndShoot extends Command {
  public final Rotation2d angleDeadband = Rotation2d.fromDegrees(1.5);
  HoodSubsystem hood;
  TargetingSubsystem targeting;
  ShooterSubsystem shooter;
  MagicFloorSubsystem floor;
  TransporterSubsystem transporter;
  CommandSwerveDrivetrain drivetrain;
  Rotation2d tunableHoodAngle = new Rotation2d();
  double tunableRPS = 0;
  Rotation2d tunableHeadingOffset = new Rotation2d();
  double tunableTransport = 0;

  PIDController rotationController = new PIDController(65, 0, 0.01);

  public boolean hasSpunUp = false;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  public SimpleAlignAndShoot(
      HoodSubsystem hoodSubsystem,
      TargetingSubsystem targetingSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagicFloorSubsystem magicFloorSubsystem,
      TransporterSubsystem transporterSubsystem,
      CommandSwerveDrivetrain drivetrainSubsystem,
      Rotation2d hoodAngle,
      double rps, Rotation2d headingOffset, double transportVoltageOffset) {
    hood = hoodSubsystem;
    targeting = targetingSubsystem;
    shooter = shooterSubsystem;
    floor = magicFloorSubsystem;
    transporter = transporterSubsystem;
    drivetrain = drivetrainSubsystem;

    tunableHoodAngle = hoodAngle;
    tunableRPS = rps;
    tunableHeadingOffset = headingOffset;
    tunableTransport = transportVoltageOffset;

    addRequirements(hood, targeting, shooter, floor, transporter, drivetrain);
  }

  @Override
  public void initialize() {
    rotationController.setTolerance(Units.degreesToRotations(1.25));
    System.out.println(rotationController.atSetpoint());
    rotationController.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void execute() {
    rotationController.setSetpoint(targeting.getIdealRobotHeading().get().getRotations());
    //rotationController.setSetpoint(0.1056);

    double desiredRotationalRate =
        rotationController.calculate(
            drivetrain.getRobotPose().getRotation().plus(tunableHeadingOffset).getRotations(),
            targeting.getIdealRobotHeading().get().getRotations());
    
    shooter.setTargetRPS(targeting.getIdealFlywheelRPS().get() + tunableRPS);
    hood.setTargetAngle(() -> targeting.getIdealHoodAngle().get().plus(tunableHoodAngle));
    //hood.setTargetAngle(() -> Rotation2d.fromRotations(.035));

    if (rotationController.atSetpoint()) {

      if ((shooter.isAtSetpoint() || hasSpunUp) && hood.isAtSetpoint()) {
        hasSpunUp = true;
        floor.setVoltageFunction(8);
        transporter.setVoltageFunction(-6 - tunableTransport - 1.5);

      }

    } else {
      drivetrain.setControl(driveRequest.withRotationalRate(desiredRotationalRate));
    }
  }

    @Override
  public void end(boolean interrupted) {
    shooter.setTargetRPS(0);
    shooter.setVoltage(0);
    hood.setVoltage(0);
    floor.setVoltage(0);
    transporter.setVoltage(0);
  }
}
