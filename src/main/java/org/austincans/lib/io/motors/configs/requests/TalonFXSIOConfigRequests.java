package org.austincans.lib.io.motors.configs.requests;

import org.austincans.lib.io.motors.configs.TalonFXSConfig;
import org.austincans.lib.io.motors.configs.TalonFXSConfig.ExternalFeedbackConfig;
import org.austincans.lib.io.motors.configs.TalonFXSConfig.MotorArrangement;

public class TalonFXSIOConfigRequests extends TalonIOConfigRequests {
    public boolean externalFeedbackConfigChanged;
    public boolean commutationConfigChanged;

    public void setExternalFeedbackConfig(ExternalFeedbackConfig externalFeedbackConfig) {
        TalonFXSConfig talonFXSConfig = (TalonFXSConfig) config;
        configChanged |= externalFeedbackConfigChanged = !talonFXSConfig.externalFeedbackConfig.equals(externalFeedbackConfig);
        talonFXSConfig.externalFeedbackConfig = externalFeedbackConfig;
    }
    public void setCommutationConfig(MotorArrangement motorArrangement) {
        TalonFXSConfig talonFXSConfig = (TalonFXSConfig) config;
        configChanged |= commutationConfigChanged = talonFXSConfig.motorArrangement != motorArrangement;
        talonFXSConfig.motorArrangement = motorArrangement;
    }
}