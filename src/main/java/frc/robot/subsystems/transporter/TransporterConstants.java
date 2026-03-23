package frc.robot.subsystems.transporter;

import frc.robot.util.LoggedTunableNumber;

public class TransporterConstants {
  public static int leaderMotorID = 16;
  public static int followerMotorID = 11;
  public static double currentLimit = 0.0;

  public static double transportShootVoltage = -1;

  public static LoggedTunableNumber transportShootVolts =
      new LoggedTunableNumber("Transporter/TransportShootVolts", 0.0);
}
