package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.controller.Axis;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.targeting.TargetingSubsystem;

public class FaceHubWhileDriving extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final TargetingSubsystem targetingSubsystem;

  private final Axis xAxis;
  private final Axis yAxis;

  private static final double kP = 6.0;
  private static final double kI = 0.0;
  private static final double kD = 0.5;

  private static final double ANGLE_TOLERANCE = Math.toRadians(2.5);

  private final SwerveRequest.FieldCentricFacingAngle request =
      new SwerveRequest.FieldCentricFacingAngle();

  public FaceHubWhileDriving(
      CommandSwerveDrivetrain drivetrain,
      TargetingSubsystem targetingSubsystem,
      Axis xAxis,
      Axis yAxis) {

    this.drivetrain = drivetrain;
    this.targetingSubsystem = targetingSubsystem;
    this.xAxis = xAxis;
    this.yAxis = yAxis;

    request.HeadingController.setPID(kP, kI, kD);
    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    double xInput = xAxis.get();
    double yInput = yAxis.get();

    double maxSpeed = DriveConstants.MAX_TRANSLATIONAL_SPEED.in(MetersPerSecond);

    Rotation2d robotHeading = drivetrain.getHeading();

    double xSpeed =
        (xInput * robotHeading.getCos() - yInput * robotHeading.getSin()) * maxSpeed;

    double ySpeed =
        (xInput * robotHeading.getSin() + yInput * robotHeading.getCos()) * maxSpeed;


    Rotation2d targetAngle = targetingSubsystem.getIdealRobotHeading().get();

    drivetrain.setControl(
        request
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withTargetDirection(targetAngle));
  }

  @Override
  public boolean isFinished() {

    Rotation2d current = drivetrain.getHeading();
    Rotation2d target = targetingSubsystem.getIdealRobotHeading().get();

    double error =
        MathUtil.angleModulus(target.minus(current).getRadians());

    return Math.abs(error) < ANGLE_TOLERANCE;
  }
}