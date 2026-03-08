package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.SwerveDriveConfig;
import org.austincans.lib.motors.MotorFactory;
import com.pathplanner.lib.config.ModuleConfig;

public class DriveIOSwerveTalonFXPigeon2 extends DriveIOSwerve<ModuleIOTalonFXCANcoder, GyroIOPigeon2> {
    public DriveIOSwerveTalonFXPigeon2(SwerveDriveConfig swerveDriveConfig, ModuleConfig moduleConfig) {
        super(
            swerveDriveConfig,
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            GyroFactory.createPigeon2();
        );
    }
}
