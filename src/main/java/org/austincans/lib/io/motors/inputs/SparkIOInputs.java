package org.austincans.lib.io.motors.inputs;

public class SparkIOInputs extends MotorIOInputs {
    public boolean activeFaultMotorType;
    public boolean activeFaultSensor;
    public boolean activeFaultCan;
    public boolean activeFaultTemperature;
    public boolean activeFaultGateDriver;
    public boolean activeFaultEscEeprom;
    public boolean activeFaultFirmware;

    public boolean stickyFaultMotorType;
    public boolean stickyFaultSensor;
    public boolean stickyFaultCan;
    public boolean stickyFaultTemperature;
    public boolean stickyFaultGateDriver;
    public boolean stickyFaultEscEeprom;
    public boolean stickyFaultFirmware;
    public boolean stickyFaultOther;
}
