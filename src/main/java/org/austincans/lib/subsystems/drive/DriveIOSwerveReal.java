package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIO;
import org.austincans.lib.motors.MotorIOTalonFX;

public class DriveIOSwerveTalonFXPigeon2 extends DriveIOSwerve<MotorIOTalonFX, GyroIOPigeon2> {
    public DriveIOSwerveTalonFXPigeon2(DriveConfig driveConfig) {
        super(
            driveConfig,
            MotorFactory.createSparkMax(0, false, null, null, false, null)
        );
    }
}
