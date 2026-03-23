// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.lights.LEDSegment;
import frc.robot.subsystems.lights.LightsConstants.ColorPalette;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    Logger.recordMetadata("ProjectName", "Javabot-2026"); // Set a metadata value

    if (isReal()) {
      // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Indicate if the battery is at voltage
    if (RobotController.getBatteryVoltage() > 12.3)
      LEDSegment.BatteryIndicator.setSolidColor(ColorPalette.Green);
    else LEDSegment.BatteryIndicator.setFadeAnimation(ColorPalette.Red, 1);

    // Indicate once the driver station is connected
    if (DriverStation.isDSAttached())
      LEDSegment.ConnectedIndicator.setSolidColor(ColorPalette.Orange);
    else LEDSegment.ConnectedIndicator.setSolidColor(ColorPalette.Black);

    // // Verify that all absolute encoders are connected
    // if (m_robotContainer.pivotSubsystem.isEncoderConnected())
    //   LEDSegment.PivotIndicator.setSolidColor(ColorPalette.Green);
    // else LEDSegment.PivotIndicator.setSolidColor(ColorPalette.Black);

    // if (m_robotContainer.hood.isEncoderConnected())
    //   LEDSegment.HoodIndicator.setSolidColor(ColorPalette.Green);
    // else LEDSegment.HoodIndicator.setSolidColor(ColorPalette.Black);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
