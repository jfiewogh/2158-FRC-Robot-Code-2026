package org.austincans.lib.io;

public interface ActuatorIO<InputsType, OutputsType> extends DeviceIO<InputsType> {
    public void applyOutputs(OutputsType outputs);
}