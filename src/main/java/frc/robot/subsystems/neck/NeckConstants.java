package frc.robot.subsystems.neck;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class NeckConstants {

  public static int motorID = 11;
  public static double currentLimit = 0.0;

  public static final Slot0Configs SlotConfigs =
      new Slot0Configs().withKP(0.3).withKI(0).withKD(0).withKS(.25).withKA(0).withKV(.115);

  public static TalonFXConfiguration talonFXConfig =
      new TalonFXConfiguration()
          .withSlot0(SlotConfigs)
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
}
