package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FaceHubWhileDriving;
import frc.robot.commands.SimpleAlignAndShoot;
import frc.robot.lib.controller.LogitechController;
import frc.robot.lib.controller.ThrustmasterJoystick;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodIOTalonFXS;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.roller.RollerIOTalonFX;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorIOTalonFX;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterIOTalonFX;
import frc.robot.subsystems.transporter.TransporterSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.Set;

public class RobotContainer {

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // public final Auto auto;

  public LoggedTunableNumber tunablerps = new LoggedTunableNumber("rps");
  public LoggedTunableNumber tunableHoodAngle = new LoggedTunableNumber("hood-angle");
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private final double maxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

  private final ThrustmasterJoystick leftDriveController = new ThrustmasterJoystick(0);

  private final ThrustmasterJoystick rightDriveController = new ThrustmasterJoystick(1);

  private final LogitechController operatorController = new LogitechController(2);

  public double shooterRPSOffset = 0.0;
  public double hoodAngleOffsetRotations = 0.0;

  private static final double RPS_STEP = 1.5;
  private static final double HOOD_STEP = 0.005;

  // public final PivotSubsystem pivot = new PivotSubsystem(new PivotIOTalonFX());
  public final RollerSubsystem roller = new RollerSubsystem(new RollerIOTalonFX());
  public final ShooterSubsystem shooter = new ShooterSubsystem(new ShooterIOTalonFX());
  public final HoodSubsystem hood = new HoodSubsystem(new HoodIOTalonFXS());
  public final TransporterSubsystem transporter =
      new TransporterSubsystem(new TransporterIOTalonFX());
  public final MagicFloorSubsystem magicFloor = new MagicFloorSubsystem(new MagicFloorIOTalonFX());
  public final TargetingSubsystem targeting = new TargetingSubsystem(drivetrain);

  public final VisionSubsystem vision =
      new VisionSubsystem(
          drivetrain::filterAndAddMeasurements,
          new VisionIOLimelight("limelight-left", drivetrain::getHeading),
          new VisionIOLimelight("limelight-backl", drivetrain::getHeading),
          new VisionIOLimelight("limelight-backr", drivetrain::getHeading),
          new VisionIOLimelight("limelight-right", drivetrain::getHeading));

  // public final LightsSubsystem lights = new LightsSubsystem();

  private final FaceHubWhileDriving faceHubCommand =
      new FaceHubWhileDriving(
          drivetrain, targeting, leftDriveController.getYAxis(), leftDriveController.getXAxis());

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.05)
          .withRotationalDeadband(maxAngularRate * 0.025)
          .withDriveRequestType(DriveRequestType.Velocity);

  public RobotContainer() {
    tunablerps.initDefault(0);
    tunableHoodAngle.initDefault(0);
    configureBindings();

    //   auto = new Auto(this);

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              ChassisSpeeds speeds = getDriverChassisSpeeds();

              return driveRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond);
            }));
  }

  // rollerSubsystem.setDefaultCommand(
  //     Commands.run(
  //         () -> {
  //           double xSpeed = drivetrain.getRobotSpeeds().vxMetersPerSecond;
  //           if (xSpeed > 0.2) { //&& pivotSubsystem.isDown()
  //             rollerSubsystem.runRoller(RollerConstants.intakeVoltage);
  //           } else {
  //             rollerSubsystem.runRoller(0.0);
  //           }
  //         }, drivetrain, rollerSubsystem));
  //       }

  private Command face0 =
      drivetrain.snapToAngle(
          () -> Rotation2d.fromDegrees(0), () -> getXVelocity(), () -> getYVelocity());

  private Command face90 =
      drivetrain.snapToAngle(
          () -> Rotation2d.fromDegrees(90), () -> getXVelocity(), () -> getYVelocity());

  private Command face180 =
      drivetrain.snapToAngle(
          () -> Rotation2d.fromDegrees(180), () -> getXVelocity(), () -> getYVelocity());

  private Command face270 =
      drivetrain.snapToAngle(
          () -> Rotation2d.fromDegrees(270), () -> getXVelocity(), () -> getYVelocity());

  private void configureBindings() {

    rightDriveController
        .getLeftTopLeft()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivetrain.resetPose(
                        new Pose2d(
                            drivetrain.getRobotPose().getX(),
                            drivetrain.getRobotPose().getY(),
                            drivetrain.getOperatorForwardDirection())),
                drivetrain));

    // driver bind
    // rightDriveController.getLeftThumb().onTrue(pivot.Toggle());
    rightDriveController.getTrigger().whileTrue(roller.setVoltage(12));

    rightDriveController.getBottomThumb().whileTrue(faceHubCommand);

    // Cardinal directions
    rightDriveController.getPOVUp().whileTrue(face0);
    rightDriveController.getPOVLeft().whileTrue(face90);
    rightDriveController.getPOVDown().whileTrue(face180);
    rightDriveController.getPOVRight().whileTrue(face270);

    // op binds
    operatorController.getA().whileTrue(roller.setVoltage(-12));
    // operatorController.getY().whileTrue(pivot.Crunch());

    operatorController
        .getY()
        .whileTrue(
            // reverse, unjam
            Commands.parallel(
                shooter.setVoltage(-4),
                transporter.setVoltage(4),
                magicFloor.setVoltage(-4),
                roller.setVoltage(-4)
            )
        );

    // operatorController
    //     .getLeftTrigger()
    //     .whileTrue(
    //         new AutoShootWhileBracing(
    //             drivetrain,
    //             targeting,
    //             shooter,
    //             hood,
    //             transporter,
    //             magicFloor,
    //             () -> shooterRPSOffset,
    //             () -> hoodAngleOffsetRotations));

    // operatorController
    //     .getRightTrigger()
    //     .whileTrue(
    //         new ShootWhileDriving(
    //             drivetrain,
    //             targeting,
    //             shooter,
    //             hood,
    //             transporter,
    //             magicFloor,
    //             leftDriveController.getYAxis(),
    //             leftDriveController.getXAxis(),
    //             () -> shooterRPSOffset,
    //             () -> hoodAngleOffsetRotations));

    // operatorController
    //     .getRightBumper()
    //     .whileTrue(
    //         new MediumDistanceFeed(
    //             drivetrain,
    //             targeting,
    //             shooter,
    //             hood,
    //             transporter,
    //             magicFloor,
    //             leftDriveController.getYAxis(),
    //             leftDriveController.getXAxis(),
    //             () -> shooterRPSOffset,
    //             () -> hoodAngleOffsetRotations));

    operatorController
        .getRightBumper()
        .whileTrue(
            Commands.defer(
                () -> {
                  return new SimpleAlignAndShoot(
                      hood,
                      targeting,
                      shooter,
                      magicFloor,
                      transporter,
                      drivetrain,
                      Rotation2d.fromRotations(-.0656),
                      tunablerps.get());
                },
                Set.of(hood, targeting, shooter, magicFloor, transporter, drivetrain)));

    // operatorController.getRightBumper().whileTrue(hood.setHoodAngleForever(() ->
    // Rotation2d.fromRotations(0.169)));

    operatorController
        .getLeftBumper()
        .whileTrue(hood.setHoodAngleForever(() -> HoodConstants.minHoodAngle));
    // operatorController
    //     .getLeftBumper()
    //     .whileTrue(
    //         new LongDistanceFeed(
    //             drivetrain,
    //             targeting,
    //             shooter,
    //             hood,
    //             transporter,
    //             magicFloor,
    //             leftDriveController.getYAxis(),
    //             leftDriveController.getXAxis(),
    //             () -> shooterRPSOffset,
    //             () -> hoodAngleOffsetRotations));

    // Shooter tuning
    operatorController.getDPadUp().onTrue(Commands.runOnce(() -> shooterRPSOffset += RPS_STEP));

    operatorController.getDPadDown().onTrue(Commands.runOnce(() -> shooterRPSOffset -= RPS_STEP));

    // Hood tuning
    operatorController
        .getDPadLeft()
        .onTrue(Commands.runOnce(() -> hoodAngleOffsetRotations += HOOD_STEP));

    operatorController
        .getDPadRight()
        .onTrue(Commands.runOnce(() -> hoodAngleOffsetRotations -= HOOD_STEP));

    // operatorController.getRightTrigger().whileTrue(magicFloor.setVoltage(8));

    // operatorController.getLeftBumper().whileTrue(transporter.setVoltage(-8));
    // operatorController.getRightBumper().whileTrue(shooter.setShooterRPSForever(35.0));

    // operatorController
    //     .getB()
    //     .whileTrue(hood.setHoodAngleForever(() -> Rotation2d.fromRotations(-0.1)));

    // operatorController
    //     .getX()
    //     .whileTrue(hood.setHoodAngleForever(() -> Rotation2d.fromRotations(0.05)));

    operatorController
        .getLeftTrigger()
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                    shooter.setShooterRPSCommand(() -> 35.0),
                    hood.setHoodAngle(() -> Rotation2d.fromRotations(0.05))),
                Commands.waitSeconds(0.4),
                Commands.parallel(magicFloor.setVoltage(8), transporter.setVoltage(-8))));
  }

  private ChassisSpeeds getDriverChassisSpeeds() {
    return new ChassisSpeeds(getXVelocity(), getYVelocity(), getThetaVelocity());
  }

  private double getXVelocity() {
    return DriveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond)
        * -Math.pow(leftDriveController.getYAxis().get(), 3);
  }

  private double getYVelocity() {
    return DriveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond)
        * -Math.pow(leftDriveController.getXAxis().get(), 3);
  }

  private double getThetaVelocity() {
    return DriveConstants.MAX_ROTATIONAL_SPEED.in(RadiansPerSecond)
        * -Math.pow(rightDriveController.getXAxis().get(), 3);
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
