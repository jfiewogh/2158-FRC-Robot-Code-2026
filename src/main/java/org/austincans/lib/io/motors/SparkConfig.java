    package org.austincans.lib.io.motors;

    import org.austincans.lib.io.motors.MotorConfigs.ControlConfig;
    import org.austincans.lib.io.motors.MotorConfigs.SoftLimitConfig;
    import org.austincans.lib.io.motors.MotorIO.NeutralMode;
    import org.austincans.lib.io.motors.SparkIO.SparkControlSlot;
    import org.austincans.lib.util.math.physical.Gearbox;

    import edu.wpi.first.units.measure.Angle;
    import edu.wpi.first.units.measure.AngularAcceleration;
    import edu.wpi.first.units.measure.AngularVelocity;
    import edu.wpi.first.units.measure.Current;

    public class SparkConfig {
    public boolean inverted;
    public NeutralMode neutralMode;
    public Current smartCurrentStallLimit;
    public EncoderConfig encoderConfig;
    public SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    public ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    public FollowerConfig followerConfig = new FollowerConfig();

    public static class EncoderConfig {
        public Gearbox sensorToMechanismGearbox;

        public boolean equals(EncoderConfig other) {
            return sensorToMechanismGearbox.getReduction() == other.sensorToMechanismGearbox.getReduction();
        }
    }

    public static class ClosedLoopConfig {
        public SlotConfig slot0Config = new SlotConfig(SparkControlSlot.SLOT_0);
        public SlotConfig slot1Config = new SlotConfig(SparkControlSlot.SLOT_1);
        public SlotConfig slot2Config = new SlotConfig(SparkControlSlot.SLOT_2);
        public SlotConfig slot3Config = new SlotConfig(SparkControlSlot.SLOT_3);
        
        /** 
         * TODO: Handle values that exist based on a boolean better.  
         * If positionContinuousWrap false and either positionWrappingMinInput or positionWrappingMaxInput changes,
         * then it is still equal to another ClosedLoopConfig with positionContinuousWrap false.
         */
        public boolean positionContinuousWrap;
        public Angle positionWrappingMinInput;
        public Angle positionWrappingMaxInput;

        public boolean equals(ClosedLoopConfig other) {
            return slot0Config.equals(other.slot0Config)
                && slot1Config.equals(other.slot1Config)
                && slot2Config.equals(other.slot2Config)
                && slot3Config.equals(other.slot3Config) 
                && positionContinuousWrap == other.positionContinuousWrap
                && positionWrappingMaxInput.isEquivalent(other.positionWrappingMaxInput)
                && positionWrappingMinInput.isEquivalent(other.positionWrappingMinInput);
        }
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
            return isFollower == other.isFollower
                && leaderCanId == other.leaderCanId
                && invertFollowerOutput == other.invertFollowerOutput;
        }
    }
}
