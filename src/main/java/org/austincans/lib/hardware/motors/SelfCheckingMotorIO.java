package org.austincans.lib.hardware.motors;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

/** TODO: Self checking seems to violate IO structure. Is there a way to make self checking a part of the IO and not separate? */
public interface SelfCheckingMotorIO<InputsType extends MotorIO.MotorIOInputs> extends MotorIO<InputsType> {
    public void registerSelfCheckHardware(AdvancedSubsystem subsystem, String label);
}