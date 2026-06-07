package org.austincans.lib.io.motors;

import edu.wpi.first.units.measure.Angle;

public final class MotorConfigs {
    /** Closed-Loop Control Config */
    public static class ControlConfig {
        public PIDConfig pidConfig = new PIDConfig();
        public FFConfig ffConfig = new FFConfig();
    }

    /** Proportional-Integral-Derivative Config */
    public static class PIDConfig {
        public double kP;
        public double kI;
        public double kD;
    }

    /** Feedforward Config */
    public static class FFConfig {
        public double kS;
        public double kG;
        public double kV;
        public double kA;
        public double kCos;
    }

    /** Soft limits should be in mechanism values. */
    public static class SoftLimitConfig extends Object {
        public boolean forwardSoftLimitEnabled;
        public Angle forwardSoftLimit;
        public boolean reverseSoftLimitEnabled;
        public Angle reverseSoftLimit;

        public boolean equals(SoftLimitConfig other) {
            return forwardSoftLimitEnabled == other.forwardSoftLimitEnabled
                    && forwardSoftLimit.isEquivalent(other.forwardSoftLimit)
                    && reverseSoftLimitEnabled == other.reverseSoftLimitEnabled
                    && reverseSoftLimit.isEquivalent(other.reverseSoftLimit);
        }
    }
}