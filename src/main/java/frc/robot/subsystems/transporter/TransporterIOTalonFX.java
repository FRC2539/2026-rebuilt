package frc.robot.subsystems.transporter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TransporterIOTalonFX implements TransporterIO {

    TalonFX leaderMotor = new TalonFX(TransporterConstants.leaderMotorID);
    TalonFX followerMotor = new TalonFX(TransporterConstants.followerMotorID);

    public TransporterIOTalonFX(){
    
        leaderMotor
            .getConfigurator()
                .apply(new TalonFXConfiguration()
                    .withCurrentLimits(
                        new CurrentLimitsConfigs()
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(0))
                        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));


        followerMotor.setControl(new Follower(TransporterConstants.leaderMotorID, MotorAlignmentValue.Opposed));
        leaderMotor.setVoltage(0);
    }

    @Override
    public void updateInputs(TransporterIOInputs inputs) {
        inputs.voltage = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.speed = leaderMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setVoltage(double transportVoltage) {
        leaderMotor.setVoltage(transportVoltage);
    }
}
