package org.austincans.lib.io.motors.configs.requests;

import org.austincans.lib.io.motors.configs.TalonFXConfig;
import org.austincans.lib.io.motors.configs.TalonFXConfig.FeedbackConfig;

public class TalonFXIOConfigRequests extends TalonIOConfigRequests {
    public boolean feedbackConfigChanged;

    public void setFeedbackConfig(FeedbackConfig feedbackConfig) {
        TalonFXConfig talonFXConfig = (TalonFXConfig) config;
        configChanged |= feedbackConfigChanged = !talonFXConfig.feedbackConfig.equals(feedbackConfig);
        talonFXConfig.feedbackConfig = feedbackConfig;
    }
}