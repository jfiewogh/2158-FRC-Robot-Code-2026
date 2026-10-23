package org.austincans.lib.io;

public interface ActuatorIO extends DeviceIO {
    public void applyOutputs(Outputs outputs);
}