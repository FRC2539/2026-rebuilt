// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.magicFloor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.magicFloor.MagicFloorIO.MagicFloorIOInputs;

public class MagicFloorSubsystem extends SubsystemBase {

  MagicFloorIO magicFloorIO;
  MagicFloorIOInputs inputs = new MagicFloorIOInputs();

  public MagicFloorSubsystem(MagicFloorIO magicFloorIO) {
    this.magicFloorIO = magicFloorIO;

    setDefaultCommand(null);
  }

  @Override
  public void periodic() {
    magicFloorIO.updateInputs(inputs);
    Logger.processInputs("RealOutputs/MagicFloor", inputs);
  }

  public Command setVoltage(double voltage){
    return Commands.run(
      () -> magicFloorIO.setVoltage(voltage)
    );
  }

}
