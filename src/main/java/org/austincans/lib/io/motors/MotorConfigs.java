package org.austincans.lib.io.motors;

import org.austincans.lib.util.math.FlaggedUtil;

import edu.wpi.first.units.measure.Angle;

public final class MotorConfigs {
    /** Closed-Loop Control Config */
    public static class ControlConfig {
        public PIDConfig pidConfig = new PIDConfig();
        public FFConfig ffConfig = new FFConfig();

        public boolean equals(ControlConfig other) {
            return pidConfig.equals(other.pidConfig)
                && ffConfig.equals(other.ffConfig);
        }
    }

    /** Proportional-Integral-Derivative Config */
    public static class PIDConfig {
        public double kP;
        public double kI;
        public double kD;

        public boolean equals(PIDConfig other) {
            return kP == other.kP && kI == other.kI && kD == other.kD;
        }
    }

    /** Feedforward Config */
    public static class FFConfig {
        public double kS;
        public double kG;
        public double kV;
        public double kA;
        public double kCos;

        public boolean equals(FFConfig other) {
            return kS == other.kS && kG == other.kG && kV == other.kV && kA == other.kA && kCos == other.kCos;
        }
    }

    /** Soft limits should be in mechanism values. */
    public static class SoftLimitConfig extends Object {
        public boolean forwardSoftLimitEnabled;
        public Angle forwardSoftLimit;
        public boolean reverseSoftLimitEnabled;
        public Angle reverseSoftLimit;

        public boolean equals(SoftLimitConfig other) {
            return FlaggedUtil.equals(
                    forwardSoftLimitEnabled, other.forwardSoftLimitEnabled, 
                    forwardSoftLimit, other.forwardSoftLimit)
                && FlaggedUtil.equals(
                    reverseSoftLimitEnabled, other.reverseSoftLimitEnabled, 
                    reverseSoftLimit, other.reverseSoftLimit);
        }
    }
}