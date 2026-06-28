package org.austincans.lib.io.motors.configs;

/** Proportional-Integral-Derivative Config */
public class PIDConfig {
    public double kP;
    public double kI;
    public double kD;

    public boolean equals(PIDConfig other) {
        return kP == other.kP && kI == other.kI && kD == other.kD;
    }
}