package org.austincans.lib.subsystems.drive;

import java.io.IOException;

import org.austincans.lib.configs.DriveConfig;
import org.austincans.lib.configs.DriveConfigDifferential;
import org.austincans.lib.configs.DriveConfigSwerve;
import org.austincans.lib.configs.GyroConfig;
import org.austincans.lib.configs.GyroConfigPigeon2;
import org.austincans.lib.configs.SwerveModuleConfig;
import org.austincans.lib.configs.SwerveConfig;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.SparkMaxIO;
import org.austincans.lib.motors.TalonFXIO;
import org.austincans.lib.sensors.gyros.GyroFactory;
import org.austincans.lib.subsystems.drive.swerve.ModuleFactory;
import org.austincans.lib.subsystems.drive.swerve.Swerve;
import org.austincans.lib.util.ConfigManager;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveFactory {
    public static NewDrive buildSwerveTalonFXPigeon2Swerve(SwerveConfig swerveConfig) {
        SwerveModuleConfig moduleConfig = swerveConfig.moduleConfig;
        GyroConfigPigeon2 gyroConfig = swerveConfig.gyroConfig;
        return new Swerve(
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            GyroFactory.createPigeon2(gyroConfig),
            swerveConfig);
    }

    public static Swerve buildSwerveTalonFXPigeon2SwerveFromYAML(String pathName) {
        // TODO: maybe pass ConfigManager into subsystem as optional so values can be updated while deployed.
        ConfigManager<SwerveConfig<GyroConfigPigeon2>> configManager = new ConfigManager<>(pathName, null);

    }

    public static Swerve buildSwerveTalonFXPigeon2SwerveFromTunerConstants() {
    }

    public static void createDifferentialSparkMax(DriveConfigDifferential driveConfig) {
    }
}
