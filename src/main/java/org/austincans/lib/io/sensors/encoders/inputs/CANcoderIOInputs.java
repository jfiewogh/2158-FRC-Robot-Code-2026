package org.austincans.lib.io.sensors.encoders.inputs;

public class CANcoderIOInputs extends AbsoluteEncoderIOInputs {
    public boolean faultHardware;
    public boolean faultBootDuringEnable;
    public boolean faultBadMagnet;
    public boolean faultUndervoltage;

    public boolean stickyFaultHardware;
    public boolean stickyFaultBootDuringEnable;
    public boolean stickyFaultBadMagnet;
    public boolean stickyFaultUndervoltage;
}