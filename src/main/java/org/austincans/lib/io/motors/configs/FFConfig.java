package org.austincans.lib.io.motors.configs;

/** Feedforward Config */
public class FFConfig {
    public double kS;
    public double kG;
    public double kV;
    public double kA;
    public double kCos;

    public boolean equals(FFConfig other) {
        return kS == other.kS && kG == other.kG && kV == other.kV && kA == other.kA && kCos == other.kCos;
    }
}