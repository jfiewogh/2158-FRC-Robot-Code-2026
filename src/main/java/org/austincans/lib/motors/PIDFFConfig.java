package org.austincans.lib.motors;

public record PIDFFConfig(
    double kP,
    double kI,
    double kD,
    double kS,
    double kG,
    double kV,
    double kA
) {}
