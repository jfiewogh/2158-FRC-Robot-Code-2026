package org.austincans.lib.hardware.motors.configs;

/**
 * Values as recorded in REV client or Phoenix Tuner X. 
 * Soft limit values are in rotations and after gearing.
 */
public record SoftLimitConfig(
    boolean forwardSoftLimitEnabled,
    double forwardSoftLimitRotations,
    boolean reverseSoftLimitEnabled,
    double reverseSoftLimitRotations) {}
