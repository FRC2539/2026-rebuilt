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
import frc.robot.commands.DynamicShot;
import frc.robot.commands.FieldRelativeAngleSnap;
import frc.robot.commands.HailMary;
import frc.robot.commands.LongFerry;
import frc.robot.commands.SimpleAlignAndShoot;
import frc.robot.commands.SimpleFerry;
import frc.robot.commands.StaticShot;
import frc.robot.commands.StaticShotHub;
import frc.robot.lib.controller.LogitechController;
import frc.robot.lib.controller.ThrustmasterJoystick;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodIOTalonFXS;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFX;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerIOTalonFX;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorIOTalonFX;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.neck.NeckIOTalonFX;
import frc.robot.subsystems.neck.NeckSubsystem;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterIOTalonFX;
import frc.robot.subsystems.transporter.TransporterSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotContainer {

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final Auto auto;

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private final double maxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

  private final ThrustmasterJoystick leftDriveController = new ThrustmasterJoystick(0);

  private final ThrustmasterJoystick rightDriveController = new ThrustmasterJoystick(1);

  private final LogitechController operatorController = new LogitechController(2);

  @AutoLogOutput public double shooterRPSOffset = 0;

  @AutoLogOutput public double hoodAngleOffset = 0;

  @AutoLogOutput public double neckRPSOffset = 0;

  @AutoLogOutput public double transporterOffset = 0;

  @AutoLogOutput public double tunableHeadingOffsetDeg = 0;

  public final PivotSubsystem pivot = new PivotSubsystem(new PivotIOTalonFX());
  public final RollerSubsystem roller = new RollerSubsystem(new RollerIOTalonFX());
  public final ShooterSubsystem shooter = new ShooterSubsystem(new ShooterIOTalonFX());
  public final HoodSubsystem hood = new HoodSubsystem(new HoodIOTalonFXS());
  public final NeckSubsystem neck = new NeckSubsystem(new NeckIOTalonFX());
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

  public final LightsSubsystem lights = new LightsSubsystem();

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.01)
          .withRotationalDeadband(maxAngularRate * 0.025)
          .withDriveRequestType(DriveRequestType.Velocity);

  public RobotContainer() {

    configureBindings();

    auto = new Auto(this);

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

  private void configureBindings() {

    leftDriveController
        .getRightTopLeft()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivetrain.resetPose(
                        new Pose2d(
                            drivetrain.getRobotPose().getX(),
                            drivetrain.getRobotPose().getY(),
                            drivetrain.getOperatorForwardDirection())),
                drivetrain));

    rightDriveController.getTrigger().whileTrue(roller.setVoltage(11));


    rightDriveController
        .getPOVUp()
        .whileTrue(
            new FieldRelativeAngleSnap(
                Rotation2d.kZero, drivetrain, () -> getXVelocity(), () -> getYVelocity()));
    rightDriveController
        .getPOVDown()
        .whileTrue(
            new FieldRelativeAngleSnap(
                Rotation2d.k180deg, drivetrain, () -> getXVelocity(), () -> getYVelocity()));
    rightDriveController
        .getPOVLeft()
        .whileTrue(
            new FieldRelativeAngleSnap(
                Rotation2d.fromRotations(.25), drivetrain, () -> getXVelocity(), () -> getYVelocity()));
    rightDriveController
        .getPOVRight()
        .whileTrue(
            new FieldRelativeAngleSnap(
                    Rotation2d.fromRotations(-.25), drivetrain, () -> getXVelocity(), () -> getYVelocity()));

    leftDriveController
        .getTrigger()
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
                      neck,
                      Rotation2d.fromRotations(0),
                      shooterRPSOffset,
                      Rotation2d.fromDegrees(tunableHeadingOffsetDeg),
                      neckRPSOffset,
                      transporterOffset, () -> getXVelocity(), () -> getYVelocity());
                },
                Set.of(hood, targeting, shooter, magicFloor, transporter, drivetrain, neck)));

    leftDriveController
        .getBottomThumb()
        .whileTrue(
            Commands.defer(
                () -> {
                  return new HailMary(
                      hood,
                      targeting,
                      shooter,
                      magicFloor,
                      transporter,
                      drivetrain,
                      neck,
                      Rotation2d.fromRotations(hoodAngleOffset),
                      shooterRPSOffset,
                      Rotation2d.fromDegrees(tunableHeadingOffsetDeg),
                      neckRPSOffset,
                      transporterOffset, () -> getXVelocity(), () -> getYVelocity());
                },
                Set.of(hood, targeting, shooter, magicFloor, transporter, drivetrain, neck)));


    // rightDriveController
    //     .getLeftThumb()
    //     .whileTrue(
    //         Commands.defer(
    //             () -> {
    //               return new StaticShot(
    //                   hood,
    //                   targeting,
    //                   shooter,
    //                   magicFloor,
    //                   transporter,
    //                   drivetrain,
    //                   neck,
    //                   Rotation2d.fromRotations(hoodAngleOffset),
    //                   shooterRPSOffset,
    //                   neckRPSOffset,
    //                   transporterOffset);
    //             },
               // Set.of(hood, targeting, shooter, magicFloor, transporter, drivetrain, neck)));
    rightDriveController.getBottomThumb().whileTrue(Commands.defer(
                () -> {
                  return new DynamicShot(
                      hood,
                      targeting,
                      shooter,
                      magicFloor,
                      transporter,
                      drivetrain,
                      neck,
                      Rotation2d.fromRotations(hoodAngleOffset),
                      shooterRPSOffset,
                      neckRPSOffset,
                      transporterOffset);
                },
                Set.of(hood, targeting, shooter, magicFloor, transporter, drivetrain, neck)));
    // rightDriveController
    //     .getRightThumb()
    //     .whileTrue(
    //         Commands.defer(
    //             () -> {
    //               return new StaticShot(
    //                   hood,
    //                   targeting,
    //                   shooter,
    //                   magicFloor,
    //                   transporter,
    //                   drivetrain,
    //                   neck,
    //                   Rotation2d.fromRotations(hoodAngleOffset),
    //                   shooterRPSOffset,
    //                   neckRPSOffset,
    //                   transporterOffset);
    //             },
    //             Set.of(hood, targeting, shooter, magicFloor, transporter, drivetrain, neck)));
    // op binds
    operatorController.getX().whileTrue(roller.setVoltage(-12));
    operatorController.getB().whileTrue(transporter.setVoltage(3));
    operatorController.getY().whileTrue(shooter.setShooterRPSForever(-30));
    // operatorController.getY().onTrue(Commands.defer(() -> pivot.toggleIntake(), Set.of(pivot)));
    // operatorController.getA().onTrue(pivot.PutDown());
    // operatorController.getY().onTrue(pivot.PullUp());

    operatorController
        .getRightBumper()
        .whileTrue(
            new SimpleFerry(
                hood,
                targeting,
                shooter,
                magicFloor,
                transporter,
                drivetrain, neck,
                () -> getXVelocity(),
                () -> getYVelocity()));
    operatorController
        .getLeftBumper()
        .whileTrue(
            new LongFerry(
                hood,
                targeting,
                shooter,
                magicFloor,
                transporter,
                drivetrain, neck,
                () -> getXVelocity(),
                () -> getYVelocity()));

    operatorController.getDPadLeft().onTrue(Commands.runOnce(() -> shooterRPSOffset += 1));

    operatorController.getDPadRight().onTrue(Commands.runOnce(() -> shooterRPSOffset -= 1));

    operatorController
        .getRightTrigger()
        .onTrue(Commands.defer(() -> pivot.feather(), Set.of(pivot)));//.onFalse(roller.setVoltage(0));

    operatorController
        .getLeftTrigger()
        .onTrue(Commands.defer(() -> pivot.toggleIntake(), Set.of(pivot)));
    operatorController.getA().whileTrue(roller.setVoltage(12));

    leftDriveController
        .getLeftBottomMiddle()
        .onTrue(Commands.runOnce(() -> transporterOffset += .5));
    leftDriveController
        .getLeftBottomRight()
        .onTrue(Commands.runOnce(() -> transporterOffset -= .5));

    // rightDriveController.getLeftBottomMiddle().onTrue(Commands.runOnce(() -> tunableHeadingOffsetDeg += 1)); //CCW +
    // rightDriveController.getLeftBottomRight().onTrue(Commands.runOnce(() -> tunableHeadingOffsetDeg -= 1));

    leftDriveController.getLeftThumb().whileTrue(new StaticShot(hood, targeting, shooter, magicFloor, transporter, drivetrain, neck));

    rightDriveController.getLeftBottomRight().onTrue(hood.setHoodAngleForever(() -> Rotation2d.fromRotations(.101)));
    leftDriveController.getLeftTopMiddle().onTrue(Commands.defer(() -> pivot.toggleIntake(), Set.of(pivot)));
    // hood tuning
    operatorController.getDPadUp().onTrue(Commands.runOnce(() -> hoodAngleOffset += .02));

    operatorController.getDPadDown().onTrue(Commands.runOnce(() -> hoodAngleOffset -= .02));
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
    return auto.getAutoCommand();
  }
}
