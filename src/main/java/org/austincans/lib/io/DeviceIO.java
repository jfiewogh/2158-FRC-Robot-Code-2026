package org.austincans.lib.io;

public interface DeviceIO<InputsType extends DeviceIO.DeviceIOInputs> {
    /** TODO: activeFaults only useful for real, not sim or fake */
    public static class DeviceIOInputs {
        public final String[] activeFaults = new String[0];
    }

    public void updateInputs(InputsType inputs);
}