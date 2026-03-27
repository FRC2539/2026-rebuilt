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
import frc.robot.commands.LongDistanceFeed;
import frc.robot.commands.MediumDistanceFeed;
import frc.robot.commands.SimpleAlignAndShoot;
import frc.robot.commands.SimpleFerry;
import frc.robot.lib.controller.LogitechController;
import frc.robot.lib.controller.ThrustmasterJoystick;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.hood.HoodIOTalonFXS;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFX;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerIOTalonFX;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorIOTalonFX;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
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

  @AutoLogOutput
  public double shooterRPSOffset = 0;

  @AutoLogOutput
  public Rotation2d hoodAngleOffset = Rotation2d.kZero;

  @AutoLogOutput
  public Rotation2d headingOffset = Rotation2d.kZero;


  public final PivotSubsystem pivot = new PivotSubsystem(new PivotIOTalonFX());
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

    leftDriveController
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

    rightDriveController.getTrigger().whileTrue(roller.setVoltage(12));

    // Cardinal directions
    rightDriveController.getPOVUp().whileTrue(face0);
    rightDriveController.getPOVLeft().whileTrue(face90);
    rightDriveController.getPOVDown().whileTrue(face180);
    rightDriveController.getPOVRight().whileTrue(face270);

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
                      hoodAngleOffset,
                      shooterRPSOffset, headingOffset);
                },
                Set.of(hood, targeting, shooter, magicFloor, transporter, drivetrain)));

    // op binds
    operatorController.getX().whileTrue(roller.setVoltage(-12));
    operatorController.getB().whileTrue(transporter.setVoltage(3));

    
   // operatorController.getY().onTrue(pivot.toggleIntake());
    // operatorController.getA().onTrue(pivot.PutDown());
    // operatorController.getY().onTrue(pivot.PullUp());

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
    //             () -> hoodAngleOffset.getRotations()));

    operatorController.getRightBumper().whileTrue(new SimpleFerry(hood, targeting, shooter, magicFloor, transporter, drivetrain, () -> getXVelocity(), () -> getYVelocity()));


    
    operatorController
        .getLeftBumper()
        .whileTrue(
            new LongDistanceFeed(
                drivetrain,
                targeting,
                shooter,
                hood,
                transporter,
                magicFloor,
                leftDriveController.getYAxis(),
                leftDriveController.getXAxis(),
                () -> shooterRPSOffset,
                () -> hoodAngleOffset.getRotations()));

    operatorController.getDPadUp().onTrue(Commands.runOnce(() -> shooterRPSOffset += 1));

    operatorController.getDPadDown().onTrue(Commands.runOnce(() -> shooterRPSOffset -= 1));

    // operatorController.getLeftTrigger().whileTrue(pivot.setVoltage(-2));
    // operatorController.getRightTrigger().whileTrue(pivot.setVoltage(2));

    leftDriveController.getLeftBottomMiddle().whileTrue(pivot.setVoltage(-2));
    rightDriveController.getLeftBottomRight().whileTrue(pivot.setVoltage(2));
    operatorController.getLeftTrigger().whileTrue(roller.setVoltage(12));
    operatorController.getA().onTrue(pivot.setVoltage(0));

    // Hood tuning
    operatorController
        .getDPadLeft()
        .onTrue(Commands.runOnce(() -> hoodAngleOffset.plus(Rotation2d.fromRotations(.005))));

    operatorController
        .getDPadRight()
        .onTrue(Commands.runOnce(() -> hoodAngleOffset.minus(Rotation2d.fromRotations(.005))));
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
