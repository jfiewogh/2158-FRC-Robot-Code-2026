package org.austincans.lib.io;

public interface DeviceIO {
    public Inputs getInputs();

    // TODO: this should be part of outputs
    public default void clearStickyFaults() {}
}