package org.austincans.lib.io.motors;

public class TalonFXSConfig extends TalonConfig {
    public FeedbackSensorSource feedbackSensorSource;

    public enum FeedbackSensorSource {
        COMMUTATION,
        REMOTE_CANCODER,
        FUSED_CANCODER
    }
}
