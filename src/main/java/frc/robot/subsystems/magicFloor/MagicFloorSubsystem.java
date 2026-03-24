// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.magicFloor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MagicFloorSubsystem extends SubsystemBase {

  MagicFloorIO magicFloorIO;
  MagicFloorIOInputsAutoLogged inputs = new MagicFloorIOInputsAutoLogged();

  public MagicFloorSubsystem(MagicFloorIO magicFloorIO) {
    this.magicFloorIO = magicFloorIO;

    setDefaultCommand(setVoltage(0));
  }

  @Override
  public void periodic() {
    magicFloorIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/MagicFloor", inputs);
  }

  public Command setVoltage(double voltage) {
    return Commands.run(() -> magicFloorIO.setVoltage(voltage), this);
  }

  public void setVoltageFunction(double voltage) {
    magicFloorIO.setVoltage(voltage);
  }
}
