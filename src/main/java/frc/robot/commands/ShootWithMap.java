package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.neck.NeckSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;

public class ShootWithMap extends Command {
  public final Rotation2d angleDeadband = Rotation2d.fromDegrees(1.5);
  HoodSubsystem hood;
  TargetingSubsystem targeting;
  ShooterSubsystem shooter;
  MagicFloorSubsystem floor;
  TransporterSubsystem transporter;
  NeckSubsystem neck;
  Rotation2d tunableHoodAngle = new Rotation2d();
  double tunableRPS = 0;
  Rotation2d tunableHeadingOffset = new Rotation2d();
  double tunableTransport = 0;
  public double tunableNeckRPS = 0;


  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  public Timer transportPulseTimer = new Timer();

  public Timer floorPulseTimer = new Timer();
  PIDController rotationController = new PIDController(50, 0, 0.1);

  public boolean hasSpunUp = false;
  public boolean neckIsReady = false;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  public ShootWithMap(
      HoodSubsystem hoodSubsystem,
      TargetingSubsystem targetingSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagicFloorSubsystem magicFloorSubsystem,
      TransporterSubsystem transporterSubsystem,
      NeckSubsystem neckSubsystem,
      Rotation2d hoodAngle,
      double rps,
      Rotation2d headingOffset,
      double neckRPS,
      double transportVoltageOffset, DoubleSupplier xVel, DoubleSupplier yVel) {
    hood = hoodSubsystem;
    targeting = targetingSubsystem;
    shooter = shooterSubsystem;
    floor = magicFloorSubsystem;
    transporter = transporterSubsystem;
    neck = neckSubsystem;
    tunableHoodAngle = hoodAngle;
    tunableRPS = rps;
    tunableHeadingOffset = headingOffset;
    tunableTransport = transportVoltageOffset;
    tunableNeckRPS = neckRPS;
    xSupplier = xVel;
    ySupplier = yVel;

    addRequirements(hood, targeting, shooter, floor, transporter, neck);
  }

  @Override
  public void initialize() {
    transportPulseTimer.reset();
    transportPulseTimer.start();

    floorPulseTimer.reset();
    floorPulseTimer.start();

    rotationController.setTolerance(Units.degreesToRotations(1.25));
    System.out.println(rotationController.atSetpoint());
    rotationController.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void execute() {

    // if (transportPulseTimer.get() > .3) { // backspin transport to clear jammed balls out
    //   transporter.setVoltageFunction(3.5);
    // } else {
    //   transporter.setVoltageFunction(0);
    // }


    System.out.println(
        "shooter:"
            + shooter.isAtSetpoint()
            + "hood:"
            + hood.isAtSetpoint()
            + "neck:"
            + neck.isAtSetpoint());


    rotationController.setSetpoint(targeting.getIdealRobotHeading().get().getRotations());

    double desiredRotationalRate = 0;
        // rotationController.calculate(
        //     drivetrain.getRobotPose().getRotation().plus(tunableHeadingOffset).getRotations(),
        //     targeting.getIdealRobotHeading().get().getRotations());

    shooter.setTargetRPS(targeting.getIdealFlywheelRPS().get() + tunableRPS);
    neck.setTargetRPS(targeting.getIdealNeckRPS().get() + tunableNeckRPS);
    hood.setTargetAngle(() -> targeting.getIdealHoodAngle().get().plus(tunableHoodAngle));

    if (true) {

      if ((shooter.isAtSetpoint() || hasSpunUp)
          && hood.isAtSetpoint()
          && (neck.isAtSetpoint() || neckIsReady)) {
        neckIsReady = true;
        hasSpunUp = true;
        floor.setVoltageFunction(8);
        transporter.setVoltageFunction(-9.5 - tunableTransport);
        //pulseFloor();
      }

    } else {
      //rivetrain.setControl(driveRequest.withRotationalRate(desiredRotationalRate).withVelocityX(xSupplier.getAsDouble()).withVelocityY(ySupplier.getAsDouble()));
    }

    //drivetrain.setControl(driveRequest.withRotationalRate(desiredRotationalRate).withVelocityX(xSupplier.getAsDouble()).withVelocityY(ySupplier.getAsDouble()));
  }


  // public void pulseFloor() {
  //   double desiredVoltage = -9.5;
  //   if (floorPulseTimer.get() > .3) {
  //     floorPulseTimer.reset();
  //     desiredVoltage = 0;
  //   } else {
  //     desiredVoltage = -9.5;
  //   }
  //   transporter.setVoltageFunction(desiredVoltage);
  // }

  @Override
  public void end(boolean interrupted) {
    shooter.setTargetRPS(0);
    shooter.setVoltage(0);
    hood.setVoltage(0);
    floor.setVoltage(0);
    transporter.setVoltage(0);
  }
}
