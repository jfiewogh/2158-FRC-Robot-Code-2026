package org.austincans.lib.io.motors;

public class TalonFXConfig extends TalonConfig<TalonFXConfig.FeedbackSensorSource> {
    /** 
     * <p>IMPORTANT: Remote and Fused CANcoder only works if on same CAN Bus as TalonFX motor</p>
     * <p>Add more enum constants if necessary</p>
     */
    public enum FeedbackSensorSource implements TalonConfig.FeedbackSensorSource {
        ROTOR,
        REMOTE_CANCODER,
        FUSED_CANCODER
    }
}
