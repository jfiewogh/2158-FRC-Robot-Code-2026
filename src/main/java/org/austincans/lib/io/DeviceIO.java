package org.austincans.lib.io;

public interface DeviceIO<InputsType extends DeviceIO.DeviceIOInputs> {
    public static abstract class DeviceIOInputs {
        private boolean connected;

        /** Should only be updated in the IO */
        public final void setConnected(boolean connected) {
            this.connected = connected;
        }

        public final boolean isConnected() {
            return connected;
        }
    }

    public void updateInputs(InputsType inputs);
}
