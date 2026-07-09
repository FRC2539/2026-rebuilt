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
      new ShootingParameters(new Rotation2d(), HoodConstants.minHoodAngle, 0.0, 0.0);

  @AutoLogOutput public Pose2d hubPosition;
  @AutoLogOutput public Pose2d desiredRobotPosition;
  @AutoLogOutput public double realDistance = 0;

  @AutoLogOutput public Rotation2d targetRobotAngle;
  @AutoLogOutput public double targetHoodAngle;
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
            1.2,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations()),
            31.5, // 30.5
            55.0)); 
    TargetingConstants.hubShotMap.put(
        2.56,
        new ShotSettings( // -.0654
            1.2,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .090508),// .068181
            34.0 + 1,
            55.0)); 
    TargetingConstants.hubShotMap.put(
        3.127,
        new ShotSettings(
            1.2,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .15),//+0.06
            36.0 - .5,
            55.0)); 
    TargetingConstants.hubShotMap.put(
        3.568,
        new ShotSettings(
            1.4,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .165),//+0.05
            37.0, // 36.0
            55.0));
    TargetingConstants.hubShotMap.put(
        4.6,
        new ShotSettings(
            1.5,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .160),//+0.12
            42.5,
            55.0));


        TargetingConstants.hubShotMap.put(
        5.6,
        new ShotSettings(
            1.7,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .12),//+0.12
            46.0,
            55.0));

        TargetingConstants.hubShotMap.put(
        6.6,
        new ShotSettings(
            1.7,
            Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + .12),//+0.12
            50.0,
            55.0));

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


    calculatedParams = calculateShotSOTM(robotPose, drivetrain.getFieldSpeeds(), hubPosition.getTranslation());
    
    
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

    desiredRobotPosition = new Pose2d(robotPose.getX(), robotPose.getY(), neededHeading);
    if (mapValues == null) {
      return calculatedParams; // Return last known good params
    }

    targetHoodAngle = mapValues.hoodAngle().getRotations();
    targetFlywheelRPS = mapValues.wheelRPS();
    targetNeckRPS = mapValues.neckRPS();
    return new ShootingParameters(
        neededHeading, mapValues.hoodAngle(), mapValues.wheelRPS(), targetNeckRPS);
  }

  public ShootingParameters calculateShotSOTM(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation2d targetPose) {

    Translation2d filteredRobotVelocity =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    Translation2d futureRobotPose =
        robotPose
            .getTranslation()
            .plus(filteredRobotVelocity.times(TargetingConstants.estimatedShotLatency));

    Translation2d realDisplacementToHub = targetPose.minus(futureRobotPose);

    realDistance = realDisplacementToHub.getNorm();

    // Rotation2d neededHeading =
    //     realDisplacementToHub
    //         .getAngle()
    //         .plus(Rotation2d.k180deg); // shooter is facing backwards, need to offset by 180 degrees

    ShotSettings mapValues = TargetingConstants.hubShotMap.get(realDistance);
    double estimatedFlightTime = mapValues.timeOfFlight();

    Translation2d virtualTarget = targetPose;

    double virtualDistance = realDistance;

    for (int i = 0; i < 4; i++) {
      virtualTarget = targetPose.minus(filteredRobotVelocity.times(estimatedFlightTime));

      virtualDistance = futureRobotPose.getDistance(virtualTarget);

      double newFlightTime = TargetingConstants.hubShotMap.get(virtualDistance).timeOfFlight();

      if (Math.abs(newFlightTime - estimatedFlightTime) < 0.005) break;

      estimatedFlightTime = newFlightTime;
    }

    Translation2d adjustedAimingVector = realDisplacementToHub.minus(filteredRobotVelocity.times(estimatedFlightTime));

    Rotation2d neededHeading = adjustedAimingVector.getAngle().plus(Rotation2d.k180deg);

    mapValues = TargetingConstants.hubShotMap.get(virtualDistance);

    targetHoodAngle = mapValues.hoodAngle().getRotations();
    targetFlywheelRPS = mapValues.wheelRPS();
    targetNeckRPS = mapValues.neckRPS();

    desiredRobotPosition = new Pose2d(robotPose.getX(), robotPose.getY(), neededHeading);

    return new ShootingParameters(
        neededHeading, mapValues.hoodAngle(), mapValues.wheelRPS(), targetNeckRPS);
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
