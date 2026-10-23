package org.austincans.lib.io.sensors.gyros.inputs;

public class Pigeon2IOInputs extends GyroIOInputs {
    public boolean hasCommunication;
    public boolean faultHardware;
    public boolean faultBootDuringEnable;
    public boolean faultBootIntoMotion;
    public boolean faultBootupAccelerometer;
    public boolean faultBootupGyroscope;

    public boolean stickyFaultHardware;
    public boolean stickyFaultBootDuringEnable;
    public boolean stickyFaultBootIntoMotion;
    public boolean stickyFaultBootupAccelerometer;
    public boolean stickyFaultBootupGyroscope;
}
