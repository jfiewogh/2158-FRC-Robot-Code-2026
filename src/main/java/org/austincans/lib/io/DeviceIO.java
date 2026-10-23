package org.austincans.lib.io;

public interface DeviceIO {
    public void updateInputs(Inputs inputs);

    public default void clearStickyFaults() {}
}