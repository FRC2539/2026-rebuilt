package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private Consumer<PoseEstimate> consumer;

  public VisionSubsystem(Consumer<PoseEstimate> consumer, VisionIO... visionIO) {

    this.io = visionIO;
    this.consumer = consumer;

    this.inputs = new VisionIOInputsAutoLogged[io.length];

    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {

    double lowestAvgDistance = Double.POSITIVE_INFINITY;
    PoseEstimate bestPoseEstimate = null;

    for (int i = 0; i < io.length; i++) {

      io[i].updateInputs(inputs[i]);

      Logger.processInputs("Vision/Camera" + i, inputs[i]);

      PoseEstimate estimate = io[i].getPoseEstimateMT2();

      if (LimelightHelpers.validPoseEstimate(estimate)) {

        if (estimate.avgTagDist < lowestAvgDistance) {
          lowestAvgDistance = estimate.avgTagDist;
          bestPoseEstimate = estimate;
        }
      }
    }

    if (bestPoseEstimate != null) {
      consumer.accept(bestPoseEstimate);
    }

    for (VisionIOInputsAutoLogged input : inputs) {
      Logger.processInputs("RealOutputs/Vision", input);
    }
  }
}
