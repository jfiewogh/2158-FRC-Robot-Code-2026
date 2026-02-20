package org.austincans.frc2158.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
    
    @AutoLog
    public class LedIOInputs{
        public LedState state;
    }

    public enum LedState{
        READYTOSHOOT,
        INTAKED,
        DRIVING,
        CLIMBING,
        SHUTTLING,
        SHOOTING,
        ALIGNED,
        NONE,
        IDLE,
        ESTOP,
        BLUE_ALLIANCE,
        RED_ALLIANCE
    }

    public void updateInputs(LedIOInputs inputs);

    public void setPattern(LedState state);
}
