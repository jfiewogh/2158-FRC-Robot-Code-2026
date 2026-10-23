package org.austincans.lib.io.motors.inputs;

import edu.wpi.first.units.measure.Current;

/** Add more fields (ex. faults) if needed */
public class TalonIOInputs extends MotorIOInputs {
    public Current supplyCurrent;
    public Current torqueCurrent;

    public boolean hasCommunication;
    public boolean faultHardware;
    public boolean faultBootDuringEnable;
    public boolean faultDeviceTemp;
    public boolean faultProcessorTemp;

    public boolean stickyFaultHardware;
    public boolean stickyFaultBootDuringEnable;
    public boolean stickyFaultUndervoltage;
    public boolean stickyFaultDeviceTemp;
    public boolean stickyFaultProcTemp;
    public boolean stickyFaultStatorCurrLimit;
    public boolean stickyFaultSupplyCurrLimit;
}
