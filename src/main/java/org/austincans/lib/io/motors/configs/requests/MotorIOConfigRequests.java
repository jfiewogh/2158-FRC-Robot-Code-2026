package org.austincans.lib.io.motors.configs.requests;

/** 
 * TODO: I feel like there is a better way to handle config requests 
 * like merging the config and the config requests together into one class. 
 * We can also try to use the TrackedValue class in util somehow
 */
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