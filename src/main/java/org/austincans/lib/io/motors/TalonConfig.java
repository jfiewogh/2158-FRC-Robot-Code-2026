package org.austincans.lib.io.motors;

import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public abstract class TalonConfig<FeedbackSensorSourceType extends TalonConfig.FeedbackSensorSource> extends MotorConfig {
    public ControlConfig slot0Config = new ControlConfig();
    public ControlConfig slot2Config = new ControlConfig();
    public ControlConfig slot1Config = new ControlConfig();

    public AngularVelocity motionMagicMechanismCruiseVelocity;
    public AngularAcceleration motionMagicMechanismAcceleration;
    public double motionMagicMechanismJerkRotationsPerSecCubed;

    /* Current Limits */
    public CurrentLimitsConfig currentLimitsConfig;

    /* Gearbox */
    public Gearbox rotorToRemoteSensor;

    /* Feedback Sensor */
    public int feedbackSensorSourceId; 
    public FeedbackSensorSourceType feedbackSensorSource;

    /** 
     * TODO: Maybe figure out a way to combine the enabled boolean with its respective value.
     * For example, statorCurrentLimitEnabled is paired with statorCurrentLimit.
     * Possibly use optional or make a monad like optional.
     */
    public static class CurrentLimitsConfig {
        public boolean statorCurrentLimitEnabled;
        public Current statorCurrentLimit;
        public boolean supplyCurrentLimitEnabled;
        public Current supplyCurrentLimit;
    }

    public static interface FeedbackSensorSource {};
}
