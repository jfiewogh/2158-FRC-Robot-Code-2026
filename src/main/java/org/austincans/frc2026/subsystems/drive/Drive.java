package org.austincans.frc2026.subsystems.drive;

import java.util.function.Consumer;

import org.austincans.lib.subsystems.drive.DriveBase;
import org.austincans.lib.subsystems.drive.swerve.DriveIOSwerve;

import edu.wpi.first.math.geometry.Pose2d;

public class Drive extends DriveBase {
    public Drive(DriveIO driveIO) {
        super(driveIO);
    }

    // public Drive(DriveIO driveIO,
    //             GyroIO gyroIO,
    //             ModuleIO flModuleIO,
    //             ModuleIO frModuleIO,
    //             ModuleIO blModuleIO,
    //             ModuleIO brModuleIO,
    //             Consumer<Pose2d> resetSimulationPoseCallBack) {
    //     super(DriveConstants.swerveDriveConfig, driveIO, gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO, resetSimulationPoseCallBack);
    }
}
