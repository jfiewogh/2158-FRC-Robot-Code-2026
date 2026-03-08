package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFX;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveFactory {
    public static SwerveDrive createSwerveTalonFXPigeon2(DriveConfig driveConfig) {
        RobotConfig robotConfig;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        return new SwerveDrive();
    }

    public static DriveBase createDifferentialSparkMax(DriveConfig driveConfig, double leftLeaderId,
            double leftFollowerId, double rightLeaderId, double rightFollowerId) {
        return new DriveBase(
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
