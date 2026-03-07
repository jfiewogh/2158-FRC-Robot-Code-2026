package org.austincans.lib.motors;

public record SoftLimitConfig(
    boolean forwardSoftLimitEnabled,
    double forwardSoftLimit,
    boolean reverseSoftLimitEnabled,
    double reverseSoftLimit
) {}
