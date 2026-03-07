package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorIOSim;

public class DriveIOSwerveSim extends DriveIOSwerve<MotorIOSim, GyroIOSim> {
    public DriveIOSwerveSim(DriveConfig driveConfig) {
        super(driveConfig);
    }
}
