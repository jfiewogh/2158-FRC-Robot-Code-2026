package org.austincans.lib.io.motors.configrequests;

public class MotorIOConfigRequests<ConfigType> {
    protected ConfigType config;
    protected boolean configChanged;

    public void setConfig(ConfigType config) {
        configChanged = true;
        this.config = config;
    }
    public ConfigType getConfig() {
        return config;
    }

    public boolean configChanged() {
        return configChanged;
    }
    public void setConfigChanged(boolean configChanged) {
        this.configChanged = configChanged;
    }
}