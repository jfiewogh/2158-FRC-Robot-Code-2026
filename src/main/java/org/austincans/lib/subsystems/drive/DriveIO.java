package org.austincans.lib.subsystems.drive;

import org.austincans.frc2026.subsystems.drive.ModuleIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveIO {
    @AutoLog
    public class DriveIOInputs {
        public ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged(); 
        public ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged(); 
        public ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged(); 
        public ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged(); 
        public Pose3d pose;
        public ChassisSpeeds velocities; // either field or robot relative
    }

    public void updateInputs(DriveIOInputs inputs);

    public void runVelocities(ChassisSpeeds velocities);

    public void resetPose2d(Pose2d pose);
}
