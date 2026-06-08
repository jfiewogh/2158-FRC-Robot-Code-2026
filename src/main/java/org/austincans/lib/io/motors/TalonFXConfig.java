package org.austincans.lib.io.motors;

public class TalonFXConfig extends TalonConfig {
    /** You still need to configure the remote feedback sensor by making an IO instance of it. */
    public FeedbackConfig feedbackConfig;

    public static class FeedbackConfig extends BaseFeedbackConfig {
        public int feedbackSensorSourceId;
        public FeedbackSensorSource feedbackSensorSource;

        public boolean equals(FeedbackConfig other) {
            return super.equals(other)
                && feedbackSensorSourceId == other.feedbackSensorSourceId 
                && feedbackSensorSource == other.feedbackSensorSource;
        }
    }

    /** 
     * <p>IMPORTANT: Remote and Fused CANcoder only works if on same CAN Bus as TalonFX motor</p>
     * <p>Add more enum constants if necessary</p>
     */
    public enum FeedbackSensorSource {
        ROTOR,
        REMOTE_CANCODER,
        FUSED_CANCODER
    }
}
