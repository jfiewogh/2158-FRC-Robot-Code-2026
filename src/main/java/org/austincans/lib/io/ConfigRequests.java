package org.austincans.lib.io;

/* 
TODO: I feel like there is a better way to handle config requests 
like merging the config and the config requests together into one class. 
We can also try to use the TrackedValue class in util somehow
*/
public abstract class ConfigRequests {
    protected Config config;
    protected boolean configChanged;

    public final void setConfig(Config config) {
        configChanged = true;
        this.config = config;
    }
    /** Get config instance and set configChanged to false */
    public final Config getConfig() {
        configChanged = false;
        return config;
    }
    public final boolean configChanged() {
        return configChanged;
    }
}