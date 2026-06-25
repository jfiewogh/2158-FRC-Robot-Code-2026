package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.configs.ControlConfig;
import org.austincans.lib.io.motors.configs.SoftLimitConfig;
import org.austincans.lib.io.motors.configs.NeutralMode;
import org.austincans.lib.io.motors.SparkIO.SparkControlSlot;
import org.austincans.lib.util.math.FlaggedUtil;
import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** 
 * <p>Configs based on AdvantageKit's Spark templates:
 * <li>https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/spark_swerve/src/main/java/frc/robot/subsystems/drive/ModuleIOSpark.java
 * <li>https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/kitbot_2026/src/main/java/frc/robot/subsystems/drive/DriveIOSpark.java
 */
public class SparkConfig {
    public boolean inverted;
    public NeutralMode neutralMode;
    public Current smartCurrentStallLimit;
    /** 
     * Duty cycle output is scaled based on this value.
     * Without compensation, it is based on the current battery voltage.
     * For example, if voltageCompensation = 12 V, then duty cycle output of 0.5 = 6 V output.
     */
    public Voltage voltageCompensation;

    public Gearbox sensorToMechanismGearbox;
    public EncoderConfig encoderConfig;
    public AbsoluteEncoderConfig absoluteEncoderConfig;
    public SignalsConfig signalsConfig;

    public SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    public ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    public FollowerConfig followerConfig = new FollowerConfig();

    public static class EncoderConfig {
        /** Must be an integer value in [8, 64]  */
        public int uvwMeasurementPeriodMs = 32;
        /** Must be value in {1, 2, 4, 8} */
        public int uvwAverageDepth = 8;

        public boolean equals(EncoderConfig other) {
            return uvwMeasurementPeriodMs == other.uvwMeasurementPeriodMs && uvwAverageDepth == other.uvwAverageDepth;
        }
    }

    public static class AbsoluteEncoderConfig {
        public boolean inverted;
        /** Must be value in {1, 2, 4, 8, 16, 32, 64, 128} */
        public int averageDepth;

        public boolean equals(AbsoluteEncoderConfig other) {
            return inverted == other.inverted && averageDepth == other.averageDepth;
        }
    }

    public static class ClosedLoopConfig {
        public FeedbackSensor feedbackSensor;

        public SlotConfig slot0Config = new SlotConfig(SparkControlSlot.SLOT_0);
        public SlotConfig slot1Config = new SlotConfig(SparkControlSlot.SLOT_1);
        public SlotConfig slot2Config = new SlotConfig(SparkControlSlot.SLOT_2);
        public SlotConfig slot3Config = new SlotConfig(SparkControlSlot.SLOT_3);
        
        public boolean positionContinuousWrap;
        public Angle positionWrappingMinInput;
        public Angle positionWrappingMaxInput;

        public boolean equals(ClosedLoopConfig other) {
            return feedbackSensor == other.feedbackSensor 
                && slot0Config.equals(other.slot0Config)
                && slot1Config.equals(other.slot1Config)
                && slot2Config.equals(other.slot2Config)
                && slot3Config.equals(other.slot3Config) 
                && FlaggedUtil.equals(
                    positionContinuousWrap, other.positionContinuousWrap, 
                    positionWrappingMaxInput.isEquivalent(other.positionWrappingMaxInput) && positionWrappingMinInput.isEquivalent(other.positionWrappingMinInput));
        }
    }

    public enum FeedbackSensor {
        PRIMARY_ENCODER,
        ABSOLUTE_ENCODER
    }

    public static class SlotConfig {
        public SparkControlSlot controlSlot;
        public ControlConfig controlConfig = new ControlConfig();
        public Angle allowedMechanismClosedLoopError;
        public AngularVelocity maxMotionMechanismCruiseVelocity;
        public AngularAcceleration maxMotionMechanismMaxAcceleration;

        public SlotConfig(SparkControlSlot controlSlot) {
            this.controlSlot = controlSlot;
        }

        public boolean equals(SlotConfig other) {
            return controlSlot == other.controlSlot
                && controlConfig.equals(other.controlConfig);
        }
    }

    public static class FollowerConfig {
        /** If true, follower should move in same direction as leader. */
        public boolean isFollower = false;
        public int leaderCanId;
        /** If true, follower should move in opposite direction of leader. */
        public boolean invertFollowerOutput;

        public boolean equals(FollowerConfig other) {
            return FlaggedUtil.equals(
                isFollower, other.isFollower,
                leaderCanId == other.leaderCanId && invertFollowerOutput == other.invertFollowerOutput);
        }
    }

    /** Default values are based on AdvantageKit's template */
    public static class SignalsConfig {
        public boolean primaryEncoderPositionAlwaysOn;
        public int primaryEncoderPositionPeriodMs;
        public boolean primaryEncoderVelocityAlwaysOn;
        public int primaryEncoderVelocityPeriodMs = 20;

        public boolean absoluteEncoderPositionAlwaysOn;
        public int absoluteEncoderPositionPeriodMs;
        public boolean absoluteEncoderVelocityAlwaysOn;
        public int absoluteEncoderVelocityPeriodMs = 20;

        public int appliedOutputPeriodMs = 20;
        public int busVoltagePeriodMs = 20;
        public int outputCurrentPeriodMs = 20;

        public boolean equals(SignalsConfig other) {
            return FlaggedUtil.equals(
                primaryEncoderPositionAlwaysOn, other.primaryEncoderPositionAlwaysOn,
                primaryEncoderPositionPeriodMs, other.primaryEncoderPositionPeriodMs)
            && FlaggedUtil.equals(
                primaryEncoderVelocityAlwaysOn, other.primaryEncoderVelocityAlwaysOn,
                primaryEncoderVelocityPeriodMs, other.primaryEncoderVelocityPeriodMs)
            && FlaggedUtil.equals(
                absoluteEncoderPositionAlwaysOn, other.absoluteEncoderPositionAlwaysOn,
                absoluteEncoderPositionPeriodMs, other.absoluteEncoderPositionPeriodMs)
            && FlaggedUtil.equals(
                absoluteEncoderVelocityAlwaysOn, other.absoluteEncoderVelocityAlwaysOn,
                absoluteEncoderVelocityPeriodMs, other.absoluteEncoderVelocityPeriodMs)
            && appliedOutputPeriodMs == other.appliedOutputPeriodMs
            && busVoltagePeriodMs == other.busVoltagePeriodMs
            && outputCurrentPeriodMs == other.outputCurrentPeriodMs;
        }
    }
}
