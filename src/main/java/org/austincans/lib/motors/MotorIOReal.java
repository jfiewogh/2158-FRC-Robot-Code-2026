package org.austincans.lib.motors;

public abstract class MotorIOReal<InputsType extends MotorIOInputs> implements SelfCheckingMotorIO<InputsType> {
    protected final int deviceId;

    protected MotorIOReal(int deviceId) {
        this.deviceId = deviceId;
    }

    public int getDeviceId() {
        return deviceId;
    }
}
