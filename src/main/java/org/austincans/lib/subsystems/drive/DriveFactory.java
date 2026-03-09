package org.austincans.lib.subsystems.drive;

import org.austincans.lib.ConfigManager;
import org.austincans.lib.configs.DriveConfig;
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
        DriveIOSwerve<ModuleIO, GyroIO> swerveIO = new DriveIOSwerve<ModuleIO, GyroIO>(
            swerveConfig,
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            ModuleFactory.createTalonFXCANCoder(moduleConfig),
            GyroFactory.);
        return new SwerveDrive(swerveIO, null, robotConfig);
    }

    public static SwerveDrive createSwerveTalonFXPigeon2FromYAML(String pathName) {
        // TODO: maybe pass ConfigManager into subsystem as optional so values can be updated while deployed.
        ConfigManager<SwerveDriveConfig> configManager = new ConfigManager<SwerveDriveConfig>(pathName, null);

    }

    public static DriveBase createDifferentialSparkMax(DriveConfig driveConfig, double leftLeaderId,
            double leftFollowerId, double rightLeaderId, double rightFollowerId) {

            
        return new DifferentialDrive<MotorIOSparkMax>(null, null, null);


            "Drive",
            new DriveIODifferential(
                driveConfig, 
                MotorFactory.createSparkMax(
                    leftLeaderId, false, null, null, false, null), 
                MotorFactory.createSparkMax(
                    leftLeaderId, false, null, null, false, null), 
                MotorFactory.createSparkMax(
                    leftLeaderId, false, null, null, false, null), 
                MotorFactory.createSparkMax(
                    leftLeaderId, false, null, null, false, null)));
    }
}
