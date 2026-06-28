package org.austincans.lib.io.motors.configs;

/**
 * It is plausible not all motors have a neutral/idle mode.
 * If true, then MotorIORequests should not have a NeutralMode.
 * You have a class that extends MotorIORequests for motors that have neutral/idle mode.
 */
public enum NeutralMode {
    BRAKE,
    COAST
}