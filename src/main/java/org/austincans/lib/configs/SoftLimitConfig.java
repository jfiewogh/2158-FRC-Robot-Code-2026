package org.austincans.lib.configs;

/**
 * Values as recorded in REV client or Phoenix Tuner X. 
 * Soft limit values are in rotations.
 * They are not affected by gear ratio or position conversion factor.
 */
public record SoftLimitConfig(
    boolean forwardSoftLimitEnabled,
    double forwardSoftLimit,
    boolean reverseSoftLimitEnabled,
    double reverseSoftLimit) {}
