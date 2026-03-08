package org.austincans.lib.subsystems.drive;

public record DriveConfig(
    double robotMassKg,
    double maxDriveSpeedMetersPerSec,
    double maxRotationSpeedRadPerSec,
    double trackWidthMeters, // distance between left and right motors
    double wheelbaseMeters, // distance between front and back motors
    ModuleConfig moduleConfig,
    double odometryFrequencyHz) {}
