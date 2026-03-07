package org.austincans.lib.subsystems.drive;

public record DriveConfig(
        double robotMassKg,
        double maxDriveSpeedMetersPerSec,
        double maxRotationSpeedRadPerSec,
        double trackWidthMeters, // distance between left and right motors
        double wheelbaseMeters, // distance between front and back motors
        double drivekP,
        double drivekI,
        double drivekD,
        double drivekS,
        double drivekV,
        double drivekT,
        double steerkP,
        double steerkI,
        double steerkD,
        double steerkS,
        double steerkV,
        double steerkA,
        double maxDriveMotorVelocityRadPerSec,
        double maxDriveMotorAccelerationRadPerSecSq,
        double maxSteerMotorVelocityRadPerSec) {
    public ModuleConfig moduleConfig() {
        return new ModuleConfig(drivekP, drivekI, drivekD, drivekS, drivekV, drivekT, steerkP, steerkI, steerkD,
                steerkS, steerkV, steerkA, maxDriveMotorVelocityRadPerSec, maxDriveMotorAccelerationRadPerSecSq, maxSteerMotorVelocityRadPerSec);
    }
}
