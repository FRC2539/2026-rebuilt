package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FaceHubWhileDriving;
import frc.robot.lib.controller.LogitechController;
import frc.robot.lib.controller.ThrustmasterJoystick;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFX;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerIOTalonFX;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class RobotContainer {

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final Auto auto;

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private final double maxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

  private final ThrustmasterJoystick leftDriveController = new ThrustmasterJoystick(0);

  private final ThrustmasterJoystick rightDriveController = new ThrustmasterJoystick(1);

  private final LogitechController operatorController = new LogitechController(2);

  public final PivotSubsystem pivotSubsystem = new PivotSubsystem(new PivotIOTalonFX());
  public final RollerSubsystem rollerSubsystem = new RollerSubsystem(new RollerIOTalonFX());

  private final FaceHubWhileDriving faceHubCommand =
      new FaceHubWhileDriving(
          drivetrain, leftDriveController.getYAxis(), leftDriveController.getXAxis());

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.05)
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
    // driver bind
    rightDriveController.getLeftThumb().onTrue(pivotSubsystem.TogglePivot());
    rightDriveController.getTrigger().whileTrue(rollerSubsystem.RunForward());

    rightDriveController.getBottomThumb().whileTrue(faceHubCommand);

    // Cardinal directions
    rightDriveController.getPOVUp().whileTrue(face0);
    rightDriveController.getPOVLeft().whileTrue(face90);
    rightDriveController.getPOVDown().whileTrue(face180);
    rightDriveController.getPOVRight().whileTrue(face270);

    // op binds
    operatorController.getA().whileTrue(rollerSubsystem.RunBackward());
    operatorController.getY().whileTrue(pivotSubsystem.CrunchSlow());
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
