package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.PIDConfig;

public record DriveConfig(
    double robotMassKg,
    double maxDriveSpeedMetersPerSec,
    double maxRotationSpeedRadPerSec,
    double trackWidthMeters, // distance between left and right motors
    double wheelbaseMeters, // distance between front and back motors
    ModuleConfig moduleConfig,
    PIDConfig drivePIDConfig,
    PIDConfig steerPIDConfig,
    double odometryFrequencyHz) {}
