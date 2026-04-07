package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.targeting.TargetingConstants.ShootingParameters;
import frc.robot.subsystems.targeting.TargetingConstants.ShotSettings;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class TargetingSubsystem extends SubsystemBase {
  private ShootingParameters calculatedParams =
      new ShootingParameters(new Rotation2d(), new Rotation2d(), 0.0, 0.0);

  @AutoLogOutput public Pose2d hubPosition;
  @AutoLogOutput public Pose2d desiredRobotPosition;
  @AutoLogOutput public double realDistance = 0;

  @AutoLogOutput public Rotation2d targetRobotAngle;
  @AutoLogOutput public Rotation2d targetHoodAngle;
  @AutoLogOutput public double targetFlywheelRPS;
  @AutoLogOutput public double targetNeckRPS;

  CommandSwerveDrivetrain drivetrain;

  public TargetingSubsystem(CommandSwerveDrivetrain dt) {
    hubPosition = TargetingConstants.blueHubPosition;

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      hubPosition = TargetingConstants.redHubPosition;
    }

    TargetingConstants.hubShotMap.put(
        1.776,
        new ShotSettings(
            1.0,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations()),
            30.5, 55.0)); // -.0656 for this and the next one
    TargetingConstants.hubShotMap.put(
        2.56,
        new ShotSettings(
            1.0, Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .02), 34.0, 55.0)); // 37
    TargetingConstants.hubShotMap.put(
        3.127,
        new ShotSettings(
            1.0,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .06),
            35.5, 45.0)); // WAS 40 RPS these 2 were zero, i'm adding the old min angle  - james
    TargetingConstants.hubShotMap.put(
        3.568,
        new ShotSettings(
            1.0,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .06),
            40.0, 55.0));
    TargetingConstants.hubShotMap.put(
        4.6,
        new ShotSettings(
            1.0,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .12),
            42.0, 55.0));


    drivetrain = dt;
  }

  @Override
  public void periodic() {

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      hubPosition = TargetingConstants.redHubPosition;
    } else {
      hubPosition = TargetingConstants.blueHubPosition;
    }
    Pose2d robotPose = drivetrain.getRobotPose();

    calculatedParams =
        calculateShot(robotPose, drivetrain.getFieldSpeeds(), hubPosition.getTranslation());
  }

  public ShootingParameters calculateShot(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation2d targetPose) {

    Translation2d realDisplacementToHub = targetPose.minus(robotPose.getTranslation());

    realDistance = realDisplacementToHub.getNorm();

    Rotation2d neededHeading =
        realDisplacementToHub
            .getAngle()
            .plus(Rotation2d.k180deg); // shooter is facing backwards, need to offset by 180 degrees

    ShotSettings mapValues = TargetingConstants.hubShotMap.get(realDistance);

    // ShotSettings mapValues = new ShotSettings(0.0, Rotation2d.kZero, 0.0);

    desiredRobotPosition = new Pose2d(robotPose.getX(), robotPose.getY(), neededHeading);
    if (mapValues == null) {
      return calculatedParams; // Return last known good params
    }

    targetHoodAngle = mapValues.hoodAngle();
    targetFlywheelRPS = mapValues.wheelRPS();
    targetNeckRPS = mapValues.neckRPS();
    return new ShootingParameters(
        neededHeading, mapValues.hoodAngle(), Math.rint(mapValues.wheelRPS()), targetNeckRPS);
  }

  public Supplier<Rotation2d> getIdealRobotHeading() {
    return () -> calculatedParams.robotHeading();
  }

  public Supplier<Rotation2d> getIdealHoodAngle() {
    return () -> calculatedParams.hoodAngle();
  }

  public Supplier<Double> getIdealFlywheelRPS() {
    return () -> calculatedParams.flywheelRPS();
  }

  public Supplier<Double> getIdealNeckRPS() {
    return () -> calculatedParams.neckRPS();
  }
}
