package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.magicFloor.MagicFloorSubsystem;
import frc.robot.subsystems.neck.NeckSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.transporter.TransporterSubsystem;

public class StaticShot extends Command {
  public final Rotation2d angleDeadband = Rotation2d.fromDegrees(1.5);
  HoodSubsystem hood;
  TargetingSubsystem targeting;
  ShooterSubsystem shooter;
  MagicFloorSubsystem floor;
  TransporterSubsystem transporter;
  CommandSwerveDrivetrain drivetrain;
  NeckSubsystem neck;
  Rotation2d tunableHoodAngle = new Rotation2d();
  double tunableRPS = 0;
  Rotation2d tunableHeadingOffset = new Rotation2d();
  double tunableTransport = 0;

  public boolean hasSpunUp = false;

  public boolean hasNeckPrepared = false;
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  public StaticShot(
      HoodSubsystem hoodSubsystem,
      TargetingSubsystem targetingSubsystem,
      ShooterSubsystem shooterSubsystem,
      MagicFloorSubsystem magicFloorSubsystem,
      TransporterSubsystem transporterSubsystem,
      CommandSwerveDrivetrain drivetrainSubsystem,
      NeckSubsystem neckSubsystem,
      Rotation2d hoodAngle,
      double rps,
      Rotation2d headingOffset,
      double transportVoltageOffset) {
    hood = hoodSubsystem;
    targeting = targetingSubsystem;
    shooter = shooterSubsystem;
    floor = magicFloorSubsystem;
    transporter = transporterSubsystem;
    drivetrain = drivetrainSubsystem;
    neck = neckSubsystem;

    tunableHoodAngle = hoodAngle;
    tunableRPS = rps;
    tunableHeadingOffset = headingOffset;
    tunableTransport = transportVoltageOffset;

    addRequirements(hood, targeting, shooter, floor, transporter, drivetrain, neck);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.setTargetRPS(42 + tunableRPS);
    neck.setTargetRPS(55);
    // hood.setTargetAngle(
    //     () -> Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + (.0656 / 1.14)));

    hood.setTargetAngle(() -> Rotation2d.fromRotations(HoodConstants.minHoodAngle.getRotations() + (.0656 / 1.14)));

    if ((shooter.isAtSetpoint() || hasSpunUp) && hood.isAtSetpoint() && (neck.isAtSetpoint() || hasNeckPrepared)) {
      hasSpunUp = true;
      hasNeckPrepared = true;
      floor.setVoltageFunction(8);
      transporter.setVoltageFunction(-7.5 - 2- tunableTransport);

    }

    drivetrain.setControl(brakeRequest);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setTargetRPS(0);
    shooter.setVoltage(0);
    hood.setVoltage(0);
    floor.setVoltage(0);
    transporter.setVoltage(0);
  }
}
