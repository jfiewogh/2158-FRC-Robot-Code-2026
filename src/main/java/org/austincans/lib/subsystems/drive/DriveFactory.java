package org.austincans.lib.subsystems.drive;

import org.austincans.lib.ConfigManager;
import org.austincans.lib.configs.DriveConfig;
import org.austincans.lib.configs.DriveConfigDifferential;
import org.austincans.lib.configs.DriveConfigSwerve;
import org.austincans.lib.configs.ModuleConfig;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkMax;
import org.austincans.lib.motors.MotorIOTalonFX;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveFactory {
    public static SwerveDrive createSwerveTalonFXPigeon2FromPathPlanner(DriveConfigSwerve swerveConfig) {
        RobotConfig robotConfig = RobotConfig.fromGUISettings();
        ModuleConfig moduleConfig = swerveConfig.moduleConfig;
        DriveIOSwerve swerveIO = new DriveIOSwerve(
            swerveConfig,
            new Module(ModuleFactory.createTalonFXCANCoder(moduleConfig)),
            new Module(ModuleFactory.createTalonFXCANCoder(moduleConfig)),
            new Module(ModuleFactory.createTalonFXCANCoder(moduleConfig)),
            new Module(ModuleFactory.createTalonFXCANCoder(moduleConfig)),
            new Gyro(GyroFactory.createPigeon2(gyroConfig)));
        return new SwerveDrive(swerveConfig, swerveIO);
    }

    public static void createSwerveTalonFXPigeon2FromYAML(String pathName) {
        // TODO: maybe pass ConfigManager into subsystem as optional so values can be updated while deployed.
        ConfigManager<DriveConfigSwerve> configManager = new ConfigManager<DriveConfigSwerve>(pathName, null);

    }

    public static void createDifferentialSparkMax(DriveConfigDifferential driveConfig) {
    }
}
