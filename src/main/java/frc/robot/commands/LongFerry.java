package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.neck.NeckSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;
import java.util.function.DoubleSupplier;

public class LongFerry extends Command {
  public final Rotation2d angleDeadband = Rotation2d.fromDegrees(1.5);
  HoodSubsystem hood;
  TargetingSubsystem targeting;
  ShooterSubsystem shooter;
  MagicFloorSubsystem floor;
  TransporterSubsystem transporter;
  CommandSwerveDrivetrain drivetrain;
  NeckSubsystem neck;
  DoubleSupplier xVel;
  DoubleSupplier yVel;

  PIDController rotationController = new PIDController(65, 0, 0.01);

  public boolean hasSpunUp = false;
  public boolean isNeckReady = false;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  public LongFerry(
      HoodSubsystem hoodSubsystem,
      TargetingSubsystem targetingSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagicFloorSubsystem magicFloorSubsystem,
      TransporterSubsystem transporterSubsystem,
      CommandSwerveDrivetrain drivetrainSubsystem, NeckSubsystem neckSubsystem,
      DoubleSupplier x,
      DoubleSupplier y) {
    hood = hoodSubsystem;
    targeting = targetingSubsystem;
    shooter = shooterSubsystem;
    floor = magicFloorSubsystem;
    transporter = transporterSubsystem;
    drivetrain = drivetrainSubsystem;
    neck = neckSubsystem;
    xVel = x;
    yVel = y;

    addRequirements(hood, targeting, shooter, floor, transporter, drivetrain, neck);
  }

  @Override
  public void initialize() {
    rotationController.setTolerance(Units.degreesToRotations(4));
    // System.out.println(rotationController.atSetpoint());
    rotationController.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void execute() {

    double targetHeading =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? .5
            : 0; // true, false.

    // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() ==
    // Alliance.Red) {
    //   rotationController.setSetpoint(Rotation2d.fromDegrees(180).getRotations());
    // } else {
    //   rotationController.setSetpoint(Rotation2d.fromDegrees(0).getRotations());?//
    // }

    rotationController.setSetpoint(targetHeading);

    // rotationController.setSetpoint(0.1056);

    double desiredRotationalRate =
        rotationController.calculate(
            drivetrain.getRobotPose().getRotation().getRotations(), targetHeading);

    
    shooter.setTargetRPS(55);
    neck.setTargetRPS(55);
    hood.setTargetAngle(() -> {return HoodConstants.maxHoodAngle;});

    // hood.setTargetAngle(() -> Rotation2d.fromRotations(.035));

    drivetrain.setControl(
        driveRequest
            .withRotationalRate(desiredRotationalRate)
            .withVelocityX(xVel.getAsDouble() / 2)
            .withVelocityY(yVel.getAsDouble() / 2));

    if (rotationController.atSetpoint()) {

      if ((shooter.isAtSetpoint() || hasSpunUp) && hood.isAtSetpoint() && (neck.isAtSetpoint() || isNeckReady)) {
        hasSpunUp = true;
        isNeckReady = true;
        floor.setVoltageFunction(8);
        transporter.setVoltageFunction(-9.5);
      }

    } else {
    }
  }
}
