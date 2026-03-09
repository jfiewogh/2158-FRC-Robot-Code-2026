package org.austincans.lib.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;

public class SwerveDrive extends Drive {
    public SwerveDrive(DriveIOSwerve<?, ?> swerveIO, HolonomicDriveController pathFollowingController, RobotConfig robotConfig) {
        super(swerveIO, pathFollowingController, robotConfig);

        RobotConfig robotConfig;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

    }
}
