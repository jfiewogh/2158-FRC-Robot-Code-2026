package org.austincans.lib.io.motors.configrequests;

import org.austincans.lib.io.motors.configs.TalonFXSConfig;
import org.austincans.lib.io.motors.configs.TalonFXSConfig.*;

public class TalonFXSIOConfigRequests extends TalonIOConfigRequests<TalonFXSConfig> {
    public boolean externalFeedbackConfigChanged;
    public boolean commutationConfigChanged;

    public void setExternalFeedbackConfig(ExternalFeedbackConfig externalFeedbackConfig) {
        configChanged |= externalFeedbackConfigChanged = !config.externalFeedbackConfig.equals(externalFeedbackConfig);
        config.externalFeedbackConfig = externalFeedbackConfig;
    }
    public void setCommutationConfig(MotorArrangement motorArrangement) {
        configChanged |= commutationConfigChanged = config.motorArrangement != motorArrangement;
        config.motorArrangement = motorArrangement;
    }
}