package org.austincans.lib.io.motors;

public class TalonFXSConfig extends TalonConfig {
    public ExternalFeedbackConfig externalFeedbackConfig;
    /** If needed, this can be moved into CommutationConfig class to add Commutation.AdvancedHallSupport */
    public MotorArrangement motorArrangement;

    public static class ExternalFeedbackConfig extends BaseFeedbackConfig {
        public int externalFeedbackSensorSourceId;
        public ExternalFeedbackSensorSource externalFeedbackSensorSource;

        public boolean equals(ExternalFeedbackConfig other) {
            return super.equals(other)
                && externalFeedbackSensorSourceId == other.externalFeedbackSensorSourceId 
                && externalFeedbackSensorSource == other.externalFeedbackSensorSource;
        }
    }

    public enum ExternalFeedbackSensorSource {
        COMMUTATION,
        REMOTE_CANCODER,
        FUSED_CANCODER
    }

    public enum MotorArrangement {
        MINION_JST,
        NEO_JST,
        VORTEX_JST,
        NEO550_JST,
        BRUSHED_AB,
        BRUSHED_AC,
        BRUSHED_BC,
        DISABLED
    }
}
