package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.input.InputSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class HubCounter extends SubsystemBase {

  private final StringPublisher colorPublisher;
  private final IntegerPublisher patternPublisher;
  private final BooleanPublisher resetPublisher;
  private final BooleanPublisher pausePublisher;

  private final IntegerSubscriber countSubscriber;
  private final IntegerSubscriber pausedCountSubscriber;

  private boolean isExternal = true;

  private final String blue = "#00c8ff";
  private final String red = "#ff0000";
  private final String black = "#000000";

  public HubCounter() {
    var table = NetworkTableInstance.getDefault().getTable("HubCounter");

    colorPublisher = table.getStringTopic("Led/Color").publish();
    patternPublisher = table.getIntegerTopic("Led/Pattern").publish();

    resetPublisher = table.getBooleanTopic("ResetCounts").publish();
    pausePublisher = table.getBooleanTopic("PauseCounting").publish();

    countSubscriber = table.getIntegerTopic("TotalCount").subscribe(0);
    pausedCountSubscriber = table.getIntegerTopic("PausedTotalCount").subscribe(0);
  }

  public void initialize() {
    resetPublisher.set(true);
  }

  public void setExternal(boolean val) {
    isExternal = val;
  }

  public enum PatternEnum {
    SOLID(0),
    PULSE(1),
    CHASE(2);

    private final int value;

    PatternEnum(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  @Override
  public void periodic() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return;

    boolean hubActive = InputSubsystem.IsHubActive();
    boolean willSwap = InputSubsystem.getWillActivitySwap();

    // Pause scoring when hub inactive
    boolean scoringPaused = !hubActive;

    // Set color
    String hubColor =
        switch (InputSubsystem.GetHubActivity()) {
          case Both, Ally -> (alliance.get() == Alliance.Blue) ? blue : red;
          case Opponent -> black;
        };

    // Set pattern
    PatternEnum pattern = PatternEnum.SOLID;

    if (willSwap) {
      pattern = PatternEnum.PULSE;
    } else if (!hubActive) {
      pattern = PatternEnum.CHASE;
    }

    // Get counts
    long scored = countSubscriber.get();
    long paused = pausedCountSubscriber.get();

    // Publish

    colorPublisher.set(hubColor);
    patternPublisher.set(pattern.getValue());
    pausePublisher.set(scoringPaused);

    // Logging
    Logger.recordOutput("HubCounter/Pattern", pattern.toString());
    Logger.recordOutput("HubCounter/Color", hubColor);
    Logger.recordOutput("HubCounter/ScoredFuel", scored);
    Logger.recordOutput("HubCounter/ScoredFuelPaused", paused);
    Logger.recordOutput("HubCounter/Paused", scoringPaused);
  }
}
