package org.austincans.lib.hardware.motors.configs;

public record PIDConfig(
    double kP,
    double kI,
    double kD) {}