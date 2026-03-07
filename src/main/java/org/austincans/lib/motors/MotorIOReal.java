package org.austincans.lib.motors;

public abstract class MotorIOReal implements MotorIO {
    protected final int deviceId;

    protected MotorIOReal(int deviceId) {
        this.deviceId = deviceId;
    }

    /** Can be used for remote CANcoder */
    public int getDeviceId() {
        return deviceId;
    }
}
