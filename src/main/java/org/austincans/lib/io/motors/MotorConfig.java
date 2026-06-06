package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.MotorIO.NeutralMode;
import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.Angle;

/** Only values part of vendor config should be included. */
public abstract class MotorConfig {
    /**
     * Sensor can be rotor or external, depending on which is chosen. Mechanism
     * represents the final output.
     */
    public Gearbox sensorToMechanismGearbox;
    public boolean inverted;
    public NeutralMode neutralMode;
    public SoftLimitConfig mechanismSoftLimitConfig;
    public boolean positionContinuousWrap;

    /** Closed-Loop Control Config */
    public static class ControlConfig {
        public PIDConfig pidConfig = new PIDConfig();
        public FFConfig ffConfig = new FFConfig();
    }

    /**
     * Proportional-Integral-Derivative Config
     * TODO: This should be moved since subsystems and other devices can also use
     * PID control, not just motors
     */
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
        public double kT;
    }

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