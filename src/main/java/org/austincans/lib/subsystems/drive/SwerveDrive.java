package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.DriveConfigSwerve;
import org.austincans.lib.configs.GyroConfigPigeon2;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveDrive extends Drive {
    public SwerveDrive(DriveConfigSwerve driveConfig, PPHolonomicDriveController pathFollowingController, RobotConfig robotConfig) {
        super(
            new DriveIOSwerve(
                driveConfig,
                ModuleFactory.createTalonFXCANCoder(driveConfig.moduleConfig),
                ModuleFactory.createTalonFXCANCoder(driveConfig.moduleConfig),
                ModuleFactory.createTalonFXCANCoder(driveConfig.moduleConfig),
                ModuleFactory.createTalonFXCANCoder(driveConfig.moduleConfig),
                GyroFactory.createPigeon2((GyroConfigPigeon2) driveConfig.gyroConfig))
            ),
            pathFollowingController, 
            robotConfig);
    }
}
