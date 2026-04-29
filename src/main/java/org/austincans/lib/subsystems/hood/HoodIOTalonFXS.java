package org.austincans.lib.subsystems.hood;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFXS;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class HoodIOTalonFXS implements HoodIO {
    private MotorIOTalonFXS motorIO;

    public HoodIOTalonFXS(HoodConfig hoodConfig) {
        motorIO = MotorFactory.createTalonFXS(hoodConfig.motorConfigTalon, hoodConfig.talonFXSConfig);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        motorIO.updateInputs(inputs.motorInputs);
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        motorIO.applyOutputs(outputs.motorOutputs);
    }

    @Override
    public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
        subsystem.registerHardware("hood", motorIO.getMotor());
    }

}
