package org.austincans.lib.io.motors;

public class TalonFXSConfig extends TalonConfig<TalonFXSConfig.TalonFXSFeedbackSensorSource> {
    public enum TalonFXSFeedbackSensorSource implements TalonConfig.FeedbackSensorSource {
        COMMUTATION,
        REMOTE_CANCODER,
        FUSED_CANCODER
    }
}
