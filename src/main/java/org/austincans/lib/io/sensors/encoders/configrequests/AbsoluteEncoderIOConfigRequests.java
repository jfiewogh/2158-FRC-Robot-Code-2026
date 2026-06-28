package org.austincans.lib.io.sensors.encoders.configrequests;

public class AbsoluteEncoderIOConfigRequests<ConfigType> {
    public ConfigType config;
    public boolean configChanged;

    public void config(ConfigType config) {
        configChanged = true;
        this.config = config;
    }
}
