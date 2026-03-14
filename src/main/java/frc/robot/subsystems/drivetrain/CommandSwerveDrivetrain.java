package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

  private static final double kSimLoopPeriod = 0.004;

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  private boolean m_hasAppliedOperatorPerspective = false;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {

    super(
        drivetrainConstants,
        250,
        VecBuilder.fill(0.02, 0.02, 0.01),
        VecBuilder.fill(0.5, 0.5, 1.0),
        modules);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  @Override
  public void periodic() {

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  @AutoLogOutput
  public Pose2d getRobotPose() {
    return getState().Pose;
  }

  @AutoLogOutput
  public Rotation2d getHeading() {
    return getRobotPose().getRotation();
  }

  public void filterAndAddMeasurements(PoseEstimate estimate) {

    if (estimate == null) return;

    boolean rejectPose = false;

    if (estimate.tagCount == 0) rejectPose = true;

    if (estimate.avgTagDist > 4.0) rejectPose = true;

    if (Math.abs(getRobotSpeeds().omegaRadiansPerSecond) > Math.toRadians(720)) rejectPose = true;

    if (rejectPose) return;

    double xyStdDev;

    if (estimate.avgTagDist < 1.5) xyStdDev = 0.08;
    else if (estimate.avgTagDist < 3) xyStdDev = 0.15;
    else xyStdDev = 0.3;

    double correctedTimestamp = estimate.timestampSeconds - (estimate.latency / 1000.0);

    addVisionMeasurement(
        estimate.pose, correctedTimestamp, VecBuilder.fill(xyStdDev, xyStdDev, 0.3));
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    super.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestamp));
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {

    super.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
  }

  @Override
  public Optional<Pose2d> samplePoseAt(double timestamp) {
    return super.samplePoseAt(Utils.fpgaToCurrentTime(timestamp));
  }

  public ChassisSpeeds getRobotSpeeds() {
    return getState().Speeds;
  }

  public SwerveModuleState[] getActualSpeeds() {
    return getState().ModuleStates;
  }

  public SwerveModuleState[] getDesiredSpeeds() {
    return getState().ModuleTargets;
  }

  private void startSimThread() {

    m_lastSimTime = Utils.getCurrentTimeSeconds();

    m_simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
