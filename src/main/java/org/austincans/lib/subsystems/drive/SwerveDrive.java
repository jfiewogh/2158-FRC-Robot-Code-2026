package org.austincans.lib.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;

public class SwerveDrive extends DriveBase {
    public SwerveDrive(DriveIOSwerve swerveIO) {
        super(swerveIO);

        RobotConfig robotConfig;
        try {
        robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

    }
}
