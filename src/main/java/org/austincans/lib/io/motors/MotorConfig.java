package org.austincans.lib.io.motors;

import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/** Values are based on the motor, not the mechanism. */
public abstract class MotorConfig {
    public int deviceId;
    public NeutralMode neutralMode;
    public boolean isInverted;
    public PIDConfig pidConfig;
    public FFConfig ffConfig;
    public SoftLimitConfig softLimitConfig;
    public AngularVelocity maxSpeed;
    public AngularAcceleration maxAcceleration;
    public Gearbox gearbox;

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
        public double kT;
    }

    public static class SoftLimitConfig {
        public boolean forwardSoftLimitEnabled;
        public Angle forwardSoftLimit;
        public boolean reverseSoftLimitEnabled;
        public Angle reverseSoftLimit;
    }

    public static enum NeutralMode {
        BRAKE, COAST
    }
}