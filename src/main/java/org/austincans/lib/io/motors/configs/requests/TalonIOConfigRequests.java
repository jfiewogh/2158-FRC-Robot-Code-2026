package org.austincans.lib.io.motors.configs.requests;

import org.austincans.lib.io.motors.configs.TalonConfig;
import org.austincans.lib.io.motors.configs.TalonConfig.*;
import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.motors.configs.ControlConfig;
import org.austincans.lib.io.motors.configs.SoftLimitConfig;

public class TalonIOConfigRequests extends ConfigRequests {
    public boolean softLimitConfigChanged;
    public boolean motorOutputConfigChanged;
    public boolean slot0ConfigChanged;
    public boolean slot1ConfigChanged;
    public boolean slot2ConfigChanged;
    public boolean closedLoopGeneralConfigChanged;
    public boolean motionMagicConfigChanged;
    public boolean currentLimitsConfigChanged;

    public void setSoftLimitConfig(SoftLimitConfig softLimitConfig) {
        TalonConfig talonConfig = (TalonConfig) config;
        configChanged |= softLimitConfigChanged = !talonConfig.softLimitConfig.equals(softLimitConfig);
        talonConfig.softLimitConfig = softLimitConfig;
    }
    public void setMotorOutputConfig(MotorOutputConfig motorOutputConfig) {
        TalonConfig talonConfig = (TalonConfig) config;
        configChanged |= motorOutputConfigChanged = !talonConfig.motorOutputConfig.equals(motorOutputConfig);
        talonConfig.motorOutputConfig = motorOutputConfig;
    }
    public void setSlot0Config(ControlConfig slotConfig) {
        TalonConfig talonConfig = (TalonConfig) config;
        configChanged |= slot0ConfigChanged = !talonConfig.slot0Config.equals(slotConfig);
        talonConfig.slot0Config = slotConfig;
    }
    public void setSlot1Config(ControlConfig slotConfig) {
        TalonConfig talonConfig = (TalonConfig) config;
        configChanged |= slot1ConfigChanged = !talonConfig.slot1Config.equals(slotConfig);
        talonConfig.slot1Config = slotConfig;
    }
    public void setSlot2Config(ControlConfig slotConfig) {
        TalonConfig talonConfig = (TalonConfig) config;
        configChanged |= slot2ConfigChanged = !talonConfig.slot2Config.equals(slotConfig);
        talonConfig.slot2Config = slotConfig;
    }
    public void setClosedLoopGeneralConfig(ClosedLoopGeneralConfig closedLoopGeneralConfig) {
        TalonConfig talonConfig = (TalonConfig) config;
        configChanged |= closedLoopGeneralConfigChanged = !talonConfig.closedLoopGeneralConfig.equals(closedLoopGeneralConfig);
        talonConfig.closedLoopGeneralConfig = closedLoopGeneralConfig;
    }
    public void setMotionMagicConfig(MotionMagicConfig motionMagicConfig) {
        TalonConfig talonConfig = (TalonConfig) config;
        configChanged |= motionMagicConfigChanged = !talonConfig.motionMagicConfig.equals(motionMagicConfig);
        talonConfig.motionMagicConfig = motionMagicConfig;
    }
    public void setCurrentLimitsConfig(CurrentLimitsConfig currentLimitsConfig) {
        TalonConfig talonConfig = (TalonConfig) config;
        configChanged |= currentLimitsConfigChanged |= !talonConfig.currentLimitsConfig.equals(currentLimitsConfig);
        talonConfig.currentLimitsConfig = currentLimitsConfig;
    }
}