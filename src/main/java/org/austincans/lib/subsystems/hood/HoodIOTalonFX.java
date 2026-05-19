package org.austincans.lib.subsystems.hood;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.TalonFXIO;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class HoodIOTalonFX implements HoodIO {
    private TalonFXIO motorIO;

    public HoodIOTalonFX(HoodConfig hoodConfig) {
        motorIO = MotorFactory.createTalonFX(hoodConfig.motorConfigTalon, hoodConfig.talonConfig);
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
        subsystem.registerHardware("Flywheel", motorIO.getMotor());
    }

}
