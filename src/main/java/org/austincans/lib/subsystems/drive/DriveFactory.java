package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFX;
import org.austincans.lib.subsystems.drive.swerve.DriveIOSwerveReal;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveFactory {
    public static DriveBase createSwerveTalonFXPigeon2(DriveConfig driveConfig) {
        return new DriveBase(
            "Drive",
            new DriveIOSwerveReal<MotorIOTalonFX, GyroIOPigeon2>(
                driveConfig,
                MotorFactory.createTalonFX(0, null, null, null)
            ));
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
