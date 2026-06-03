package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.MotorIO.ControlSlot;
import org.austincans.lib.io.motors.MotorIO.NeutralMode;
import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/** 
 * These values define the motor's initial state and behavior in response to outputs.
 */
public abstract class MotorConfig<ControlSlotType extends ControlSlot> {
    public int deviceId;
    /** Sensor can be rotor or external, depending on which is chosen */
    public Gearbox sensorToMechanismGearbox;

    /* Software properties */
    public boolean inverted;

    public NeutralMode neutralMode;

    public SoftLimitConfig softLimitConfig;
    public AngularVelocity maxSpeed;
    public AngularAcceleration maxAcceleration;

    public ControlSlotType initialSlot; 
    public ControlConfig slot0Config = new ControlConfig();
    public ControlConfig slot1Config = new ControlConfig();
    public ControlConfig slot2Config = new ControlConfig();

    /** Closed-Loop Control Config */
    public static class ControlConfig {
        public PIDConfig pidConfig = new PIDConfig();
        public FFConfig ffConfig = new FFConfig();
        public Angle allowedMechanismClosedLoopError;
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
        public double kT;
    }

    public static class SoftLimitConfig {
        public boolean forwardSoftLimitEnabled;
        public Angle forwardSoftLimit;
        public boolean reverseSoftLimitEnabled;
        public Angle reverseSoftLimit;
    }
}