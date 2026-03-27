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
  double tunablerps = 0;

  // private final SwerveRequest.FieldCentricFacingAngle driveRequest =
  //     new SwerveRequest.FieldCentricFacingAngle();

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
      double rps) {
    hood = hoodSubsystem;
    targeting = targetingSubsystem;
    shooter = shooterSubsystem;
    floor = magicFloorSubsystem;
    transporter = transporterSubsystem;
    drivetrain = drivetrainSubsystem;

    // hoodAngle = tunableHoodAngle; //this looks to be inverted, and setting to 0 since in the
    // constructor.
    tunableHoodAngle = hoodAngle;
    tunablerps = rps;

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

    // System.out.println(targeting.getIdealRobotHeading().get().getRotations());
    double desiredRotationalRate =
        rotationController.calculate(
            drivetrain.getRobotPose().getRotation().getRotations(),
            targeting.getIdealRobotHeading().get().getRotations());

    shooter.setTargetRPS(targeting.getIdealFlywheelRPS().get());
    hood.setTargetAngle(targeting.getIdealHoodAngle());

    // System.out.println(driveRequest.HeadingController.getPositionError());
    if (rotationController.atSetpoint()) {
      // shooter.setTargetRPS();
      if ((shooter.isAtSetpoint() || hasSpunUp) && hood.isAtSetpoint()) {
        hasSpunUp = true;
        floor.setVoltageFunction(8);
        transporter.setVoltageFunction(-5);

        // drivetrain.setControl(brakeRequest);
        // CommandScheduler.getInstance().schedule(floor.setVoltage(8), transporter.setVoltage(-8));
      }

    } else {
      drivetrain.setControl(driveRequest.withRotationalRate(desiredRotationalRate));
      // }
    }
  }
}
