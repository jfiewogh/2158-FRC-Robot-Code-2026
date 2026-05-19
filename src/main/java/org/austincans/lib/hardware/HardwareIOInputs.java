package org.austincans.lib.hardware;

public abstract class HardwareIOInputs {
    private boolean connected;

    /** Should only be updated in the IO */
    public void setConnected(boolean connected) {
        this.connected = connected;
    }

    public boolean isConnected() {
        return connected;
    }
}
