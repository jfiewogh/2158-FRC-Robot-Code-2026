package org.austincans.lib.subsystems.drive;

import org.austincans.lib.ConfigManager;
import org.austincans.lib.configs.DriveConfig;
import org.austincans.lib.configs.DriveConfigSwerve;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkMax;
import org.austincans.lib.motors.MotorIOTalonFX;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveFactory {
    private static <T extends ModuleIO, U extends GyroIO> SwerveDrive createSwerveDrive(DriveConfigSwerve swerveConfig) {
        DriveIOSwerve swerveIO = new DriveIOSwerve<T, U>(null, null, null);
        return new SwerveDrive(null, null, robotConfig)
        return new SwerveDrive();
    }

    public static SwerveDrive createSwerveTalonFXPigeon2(DriveConfigSwerve swerveConfig) {
        RobotConfig robotConfig;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
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

    public static DriveBase createSwerveTalonFXPigeon2FromYAML(String pathName) {
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
