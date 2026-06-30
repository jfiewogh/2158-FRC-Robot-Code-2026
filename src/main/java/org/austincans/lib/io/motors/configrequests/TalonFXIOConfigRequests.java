package org.austincans.lib.io.motors.configrequests;

import org.austincans.lib.io.motors.configs.TalonFXConfig;
import org.austincans.lib.io.motors.configs.TalonFXConfig.FeedbackConfig;

public class TalonFXIOConfigRequests extends TalonIOConfigRequests<TalonFXConfig> {
    public boolean feedbackConfigChanged;

    public void setFeedbackConfig(FeedbackConfig feedbackConfig) {
        configChanged |= feedbackConfigChanged = !config.feedbackConfig.equals(feedbackConfig);
        config.feedbackConfig = feedbackConfig;
    }
}