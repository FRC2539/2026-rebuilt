package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class FieldRelativeAngleSnap extends Command {
  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  public CommandSwerveDrivetrain drivetrain;
  public Rotation2d allianceRelativeTargetRotation;
  public PIDController rotationController = new PIDController(65, 0, 0.01);
  DoubleSupplier xVelocity;
  DoubleSupplier yVelocity;

  public FieldRelativeAngleSnap(
      Rotation2d allianceRelRot,
      CommandSwerveDrivetrain dt,
      DoubleSupplier xVel,
      DoubleSupplier yVel) {
    allianceRelativeTargetRotation = allianceRelRot;
    drivetrain = dt;
    xVelocity = xVel;
    yVelocity = yVel;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    rotationController.setTolerance(Units.degreesToRotations(1.25));
    rotationController.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void execute() {

    Rotation2d fieldRelativeTargetRotation = allianceRelativeTargetRotation;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      Rotation2d adjustedTargetRotation = allianceRelativeTargetRotation.plus(Rotation2d.k180deg);
      fieldRelativeTargetRotation =
          Rotation2d.fromRotations(
              MathUtil.inputModulus(adjustedTargetRotation.getRotations(), -.5, .5));
    }

    double desiredRotationalRate =
        rotationController.calculate(
            drivetrain.getHeading().getRotations(), fieldRelativeTargetRotation.getRotations());

    drivetrain.setControl(
        driveRequest
            .withVelocityX(xVelocity.getAsDouble())
            .withVelocityY(yVelocity.getAsDouble())
            .withRotationalRate(desiredRotationalRate));
  }
}
