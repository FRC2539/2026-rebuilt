package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

public class HailMary extends Command {
  public final Rotation2d angleDeadband = Rotation2d.fromDegrees(1.5);
  HoodSubsystem hood;
  TargetingSubsystem targeting;
  ShooterSubsystem shooter;
  MagicFloorSubsystem floor;
  TransporterSubsystem transporter;
  CommandSwerveDrivetrain drivetrain;
  NeckSubsystem neck;
  Rotation2d tunableHoodAngle = new Rotation2d();
  double tunableRPS = 0;
  Rotation2d tunableHeadingOffset = new Rotation2d();
  double tunableTransport = 0;
  public double tunableNeckRPS = 0;


  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  public Timer transportPulseTimer = new Timer();

  SlewRateLimiter xLimiter = new SlewRateLimiter(1.2);
  SlewRateLimiter yLimiter = new SlewRateLimiter(1.2);
  public Timer floorPulseTimer = new Timer();
  PIDController rotationController = new PIDController(65, 0, 0.2);

  public boolean hasSpunUp = false;
  public boolean neckIsReady = false;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  public HailMary(
      HoodSubsystem hoodSubsystem,
      TargetingSubsystem targetingSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagicFloorSubsystem magicFloorSubsystem,
      TransporterSubsystem transporterSubsystem,
      CommandSwerveDrivetrain drivetrainSubsystem,
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
    drivetrain = drivetrainSubsystem;

    neck = neckSubsystem;
    tunableHoodAngle = hoodAngle;
    tunableRPS = rps;
    tunableHeadingOffset = headingOffset;
    tunableTransport = transportVoltageOffset;
    tunableNeckRPS = neckRPS;
    xSupplier = xVel;
    ySupplier = yVel;

    addRequirements(hood, targeting, shooter, floor, transporter, drivetrain, neck);
  }

  @Override
  public void initialize() {
    transportPulseTimer.reset();
    transportPulseTimer.start();

    floorPulseTimer.reset();
    floorPulseTimer.start();

    rotationController.setTolerance(Units.degreesToRotations(1.25)); // 1.25
   // System.out.println(rotationController.atSetpoint());
    rotationController.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void execute() {

    rotationController.setSetpoint(targeting.getIdealRobotHeading().get().plus(tunableHeadingOffset).getRotations());

    double desiredRotationalRate =
        rotationController.calculate(
            drivetrain.getRobotPose().getRotation().getRotations(),
            targeting.getIdealRobotHeading().get().plus(tunableHeadingOffset).getRotations());

    shooter.setTargetRPS(targeting.getIdealFlywheelRPS().get() + tunableRPS);
    neck.setTargetRPS(targeting.getIdealNeckRPS().get() + tunableNeckRPS);
    hood.setTargetAngle(() -> targeting.getIdealHoodAngle().get().plus(tunableHoodAngle));



    floor.setVoltageFunction(8);
    transporter.setVoltageFunction(-9.5 - tunableTransport); // -9.5
        //pulseFloor();
      
    drivetrain.setControl(driveRequest.withRotationalRate(desiredRotationalRate).withVelocityX(xLimiter.calculate(xSupplier.getAsDouble()) / 2).withVelocityY(yLimiter.calculate(ySupplier.getAsDouble()) / 2));

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
