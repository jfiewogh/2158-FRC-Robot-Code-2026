package org.austincans.lib.hardware.motors;

public abstract class MotorIOReal<InputsType extends MotorIO.MotorIOInputs> implements SelfCheckingMotorIO<InputsType> {
    protected final int deviceId;

    protected MotorIOReal(int deviceId) {
        this.deviceId = deviceId;
    }

    public int getDeviceId() {
        return deviceId;
    }
}
