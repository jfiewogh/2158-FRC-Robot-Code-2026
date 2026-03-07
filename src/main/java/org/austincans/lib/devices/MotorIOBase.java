package org.austincans.lib.devices;

public abstract class MotorIOBase implements MotorIO {
    protected final int deviceId;

    public MotorIOBase(int deviceId) {
        this.deviceId = deviceId;
    }

    /** Can be used for remote CANcoder */
    public int getDeviceId() {
        return deviceId;
    }
}
