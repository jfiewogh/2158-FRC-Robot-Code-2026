package org.austincans.lib.io.motors;

public class TalonFXSConfig extends TalonConfig {
    public ExternalFeedbackConfig externalFeedbackConfig;

    public static class ExternalFeedbackConfig extends BaseFeedbackConfig {
        public int externalFeedbackSensorSourceId;
        public ExternalFeedbackSensorSource externalFeedbackSensorSource;

        public boolean equals(ExternalFeedbackConfig other) {
            return externalFeedbackSensorSourceId == other.externalFeedbackSensorSourceId 
                && externalFeedbackSensorSource == other.externalFeedbackSensorSource;
        }
    }

    public enum ExternalFeedbackSensorSource {
        COMMUTATION,
        REMOTE_CANCODER,
        FUSED_CANCODER
    }
}
