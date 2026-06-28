package org.austincans.lib.io.motors.configrequests;

public class MotorIOConfigRequests<ConfigType> {
    protected ConfigType config;
    protected boolean configChanged;

    public void setConfig(ConfigType config) {
        configChanged = true;
        this.config = config;
    }
    /** Get config instance and set configChanged to false */
    public ConfigType getConfig() {
        configChanged = false;
        return config;
    }
    public boolean configChanged() {
        return configChanged;
    }
}