package org.austincans.lib.io;

public interface ActuatorIO<InputsType extends DeviceIO.DeviceIOInputs, OutputsType> extends DeviceIO<InputsType> {
    public void applyOutputs(OutputsType outputs);
}