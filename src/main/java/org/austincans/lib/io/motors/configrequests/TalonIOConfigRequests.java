package org.austincans.lib.io.motors.configrequests;

import org.austincans.lib.io.motors.TalonConfig;
import org.austincans.lib.io.motors.TalonConfig.ClosedLoopGeneralConfig;
import org.austincans.lib.io.motors.TalonConfig.CurrentLimitsConfig;
import org.austincans.lib.io.motors.TalonConfig.MotionMagicConfig;
import org.austincans.lib.io.motors.TalonConfig.MotorOutputConfig;
import org.austincans.lib.io.motors.configs.ControlConfig;
import org.austincans.lib.io.motors.configs.SoftLimitConfig;

public class TalonIOConfigRequests<TalonConfigType extends TalonConfig> extends MotorIOConfigRequests<TalonConfigType> {
    public boolean softLimitConfigChanged;
    public boolean motorOutputConfigChanged;
    public boolean slot0ConfigChanged;
    public boolean slot1ConfigChanged;
    public boolean slot2ConfigChanged;
    public boolean closedLoopGeneralConfigChanged;
    public boolean motionMagicConfigChanged;
    public boolean currentLimitsConfigChanged;

    public void setSoftLimitConfig(SoftLimitConfig softLimitConfig) {
        configChanged |= softLimitConfigChanged = !config.softLimitConfig.equals(softLimitConfig);
        config.softLimitConfig = softLimitConfig;
    }
    public void setMotorOutputConfig(MotorOutputConfig motorOutputConfig) {
        configChanged |= motorOutputConfigChanged = !config.motorOutputConfig.equals(motorOutputConfig);
        config.motorOutputConfig = motorOutputConfig;
    }
    public void setSlot0Config(ControlConfig slotConfig) {
        configChanged |= slot0ConfigChanged = !config.slot0Config.equals(slotConfig);
        config.slot0Config = slotConfig;
    }
    public void setSlot1Config(ControlConfig slotConfig) {
        configChanged |= slot1ConfigChanged = !config.slot1Config.equals(slotConfig);
        config.slot1Config = slotConfig;
    }
    public void setSlot2Config(ControlConfig slotConfig) {
        configChanged |= slot2ConfigChanged = !config.slot2Config.equals(slotConfig);
        config.slot2Config = slotConfig;
    }
    public void setClosedLoopGeneralConfig(ClosedLoopGeneralConfig closedLoopGeneralConfig) {
        configChanged |= closedLoopGeneralConfigChanged = !config.closedLoopGeneralConfig.equals(config.closedLoopGeneralConfig);
        config.closedLoopGeneralConfig = closedLoopGeneralConfig;
    }
    public void setMotionMagicConfig(MotionMagicConfig motionMagicConfig) {
        configChanged |= motionMagicConfigChanged = !config.motionMagicConfig.equals(motionMagicConfig);
        config.motionMagicConfig = motionMagicConfig;
    }
    public void setCurrentLimitsConfig(CurrentLimitsConfig currentLimitsConfig) {
        configChanged |= currentLimitsConfigChanged |= !config.currentLimitsConfig.equals(currentLimitsConfig);
        config.currentLimitsConfig = currentLimitsConfig;
    }
}