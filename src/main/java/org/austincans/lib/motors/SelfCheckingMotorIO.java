package org.austincans.lib.motors;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public interface SelfCheckingMotorIO<InputsType extends MotorIOInputs> extends MotorIO<InputsType> {
    public void registerSelfCheckHardware(AdvancedSubsystem subsystem, String label);
}