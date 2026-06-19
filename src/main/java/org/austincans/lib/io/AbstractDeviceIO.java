package org.austincans.lib.io;

public abstract class AbstractDeviceIO<InputsType extends DeviceIO.DeviceIOInputs> implements DeviceIO<InputsType> {
    @Override
    public void updateInputs(InputsType inputs) {
        inputs.activeFaults = getActiveFaults();
    }
    
    protected abstract String[] getActiveFaults();
}