package org.austincans.lib.io.motors.outputs.control;

public enum TalonPositionControlType implements PositionControlType {
    DUTY_CYCLE,
    VOLTAGE,
    TORQUE_CURRENT_FOC,
    MOTION_MAGIC_DUTY_CYCLE,
    MOTION_MAGIC_VOLTAGE,
    MOTION_MAGIC_TORQUE_CURRENT_FOC,
    MOTION_MAGIC_EXPO_DUTY_CYCLE,
    MOTION_MAGIC_EXPO_VOLTAGE,
    MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC;
}
