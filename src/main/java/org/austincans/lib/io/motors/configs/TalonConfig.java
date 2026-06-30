package org.austincans.lib.io.motors.configs;

import org.austincans.lib.util.math.FlaggedUtil;
import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public abstract class TalonConfig {
    public MotorOutputConfig motorOutputConfig = new MotorOutputConfig();
    public SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    public ControlConfig slot0Config = new ControlConfig();
    public ControlConfig slot1Config = new ControlConfig();
    public ControlConfig slot2Config = new ControlConfig();
    public ClosedLoopGeneralConfig closedLoopGeneralConfig = new ClosedLoopGeneralConfig();
    public MotionMagicConfig motionMagicConfig = new MotionMagicConfig();
    public CurrentLimitsConfig currentLimitsConfig = new CurrentLimitsConfig();

    public abstract static class BaseFeedbackConfig {
        public Gearbox sensorToMechanismGearbox;
        public Gearbox rotorToRemoteSensorGearbox;

        public boolean equals(BaseFeedbackConfig other) {
            return sensorToMechanismGearbox.equals(other.sensorToMechanismGearbox)
                && rotorToRemoteSensorGearbox.equals(other.rotorToRemoteSensorGearbox);
        }
    } 

    public static class MotorOutputConfig {
        public boolean inverted;
        public NeutralMode neutralMode;

        public boolean equals(MotorOutputConfig other) {
            return inverted == other.inverted
                && neutralMode == other.neutralMode;
        }
    } 

    public static class ClosedLoopGeneralConfig {
        public boolean continuousWrap;

        public boolean equals(ClosedLoopGeneralConfig other) {
            return continuousWrap == other.continuousWrap;
        }
    }

    /** In mechanism values */
    public static class MotionMagicConfig {
        public AngularVelocity cruiseVelocity;
        public AngularAcceleration acceleration;
        public double jerkRotationsPerSecCubed; 
        public double expoKV;
        public double expoKA;

        public boolean equals(MotionMagicConfig other) {
            return cruiseVelocity.isEquivalent(other.cruiseVelocity)
                && acceleration.isEquivalent(other.acceleration)
                && jerkRotationsPerSecCubed == other.jerkRotationsPerSecCubed
                && expoKV == other.expoKV
                && expoKA == other.expoKA;
        }
    }

    public static class CurrentLimitsConfig {
        public boolean statorCurrentLimitEnabled;
        public Current statorCurrentLimit;

        public boolean supplyCurrentLimitEnabled;
        public Current supplyCurrentLimit;

        public boolean equals(CurrentLimitsConfig other) {
            return FlaggedUtil.equals(
                    statorCurrentLimitEnabled, other.statorCurrentLimitEnabled, 
                    statorCurrentLimit, other.statorCurrentLimit)
                && FlaggedUtil.equals(
                    supplyCurrentLimitEnabled, other.supplyCurrentLimitEnabled, 
                    supplyCurrentLimit, other.supplyCurrentLimit);
        }
    }
}
