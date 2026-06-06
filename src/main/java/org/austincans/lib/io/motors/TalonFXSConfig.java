package org.austincans.lib.io.motors;

public class TalonFXSConfig extends TalonConfig<TalonFXSConfig.FeedbackSensorSource> {
    public enum FeedbackSensorSource implements TalonConfig.FeedbackSensorSource {
        COMMUTATION,
        REMOTE_CANCODER,
        FUSED_CANCODER
    }
}
