package org.austincans.lib.io.motors.configs;

/** Closed-Loop Control Config */
public class ControlConfig {
    public PIDConfig pidConfig = new PIDConfig();
    public FFConfig ffConfig = new FFConfig();

    public boolean equals(ControlConfig other) {
        return pidConfig.equals(other.pidConfig)
            && ffConfig.equals(other.ffConfig);
    }
}