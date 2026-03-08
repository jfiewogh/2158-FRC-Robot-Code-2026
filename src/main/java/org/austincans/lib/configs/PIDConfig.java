package org.austincans.lib.configs;

public record PIDConfig(
    double kP,
    double kI,
    double kD) {}