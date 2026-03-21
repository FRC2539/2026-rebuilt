package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.controller.Axis;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;

public class FaceHubWhileDriving extends Command {

  private final CommandSwerveDrivetrain drivetrain;

  private final Axis xAxis;
  private final Axis yAxis;

  public static double fieldLengthMeters = 16.54098798984;

  public static Pose2d blueHubPosition = new Pose2d(4.6255, 4.0345, Rotation2d.kZero);

  public static Pose2d redHubPosition =
      new Pose2d(
          fieldLengthMeters - blueHubPosition.getX(), blueHubPosition.getY(), Rotation2d.kZero);

  private static final double TARGET_DEADBAND = 0.1;

  private static final double kP = 6.0;
  private static final double kI = 0.0;
  private static final double kD = 0.1;

  private final SwerveRequest.FieldCentricFacingAngle request =
      new SwerveRequest.FieldCentricFacingAngle();

  public FaceHubWhileDriving(CommandSwerveDrivetrain drivetrain, Axis xAxis, Axis yAxis) {

    this.drivetrain = drivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;

    request.HeadingController.setPID(kP, kI, kD);

    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    double xSpeed = xAxis.get() * DriveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond);

    double ySpeed = yAxis.get() * DriveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond);
    

    Pose2d hubPosition = blueHubPosition;

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      hubPosition = redHubPosition;
    }

    var pose = drivetrain.getRobotPose();

    double dx = hubPosition.getX() - pose.getX();
    double dy = hubPosition.getY() - pose.getY();

    if (Math.hypot(dx, dy) < TARGET_DEADBAND) {
      drivetrain.setControl(
          request
              .withVelocityX(xSpeed)
              .withVelocityY(ySpeed)
              .withTargetDirection(drivetrain.getHeading()));
      return;
    }

    Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx));

    drivetrain.setControl(
        request.withVelocityX(xSpeed).withVelocityY(ySpeed).withTargetDirection(targetAngle));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
