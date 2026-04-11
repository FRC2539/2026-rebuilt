package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {

  String cameraName = "";
  Supplier<Rotation2d> currentHeading;

  public VisionIOLimelight(String cameraName, Supplier<Rotation2d> currentHeading) {
    this.cameraName = cameraName;
    this.currentHeading = currentHeading;
    LimelightHelpers.Flush();

    LimelightHelpers.SetIMUMode(cameraName, 0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    updateHeading(currentHeading);
  }

  @Override
  public PoseEstimate getPoseEstimateMT2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
  }

  @Override
  public void updateHeading(Supplier<Rotation2d> currentHeading) {
    LimelightHelpers.SetRobotOrientation(
        cameraName, currentHeading.get().getDegrees(), 0, 0, 0, 0, 0);
  }
}
