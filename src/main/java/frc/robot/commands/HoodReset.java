package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;

public class HoodReset extends Command { 
    
    Timer hoodTimer;
    HoodSubsystem hoodInstance;

    public HoodReset(HoodSubsystem hood) {
        hoodInstance = hood;

        addRequirements(hoodInstance);
    }

    @Override
    public void initialize() {
        hoodTimer = new Timer();
        hoodTimer.start();
        hoodInstance.setVoltage(0);
    }

    @Override
    public void execute() {
        hoodInstance.setVoltage(-.5);
    }

    @Override
    public boolean isFinished() {
        return hoodTimer.hasElapsed(1.00);
    }

    @Override
    public void end(boolean interrupted) {
        HoodConstants.minHoodAngle = hoodInstance.getHoodPosition();
    }
}
