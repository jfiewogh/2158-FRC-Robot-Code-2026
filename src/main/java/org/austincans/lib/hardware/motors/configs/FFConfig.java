package org.austincans.lib.hardware.motors.configs;

/** Feedforward Config */
public record FFConfig(
    double kS,
    double kG,
    double kV,
    double kA,
    double kT) {}
