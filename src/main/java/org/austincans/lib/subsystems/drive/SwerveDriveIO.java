package org.austincans.lib.subsystems.drive;

import org.austincans.lib.subsystems.drive.GyroIO.GyroIOInputs;
import org.austincans.lib.subsystems.drive.ModuleIO.ModuleIOInputs;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveDriveIO {
    @AutoLog
    public class SwerveDriveIOInputs {
        public ModuleIOInputs flModuleInputs = new ModuleIOInputs();
        public ModuleIOInputs frModuleInputs = new ModuleIOInputs();
        public ModuleIOInputs blModuleInputs = new ModuleIOInputs();
        public ModuleIOInputs brModuleInputs = new ModuleIOInputs();
        public GyroIOInputs gyroInputs;
        public Pose3d pose;
        public ChassisSpeeds velocities;
    }

    public void updateInputs(SwerveDriveIOInputs inputs);

    public void runVelocities(ChassisSpeeds velocities);

    public void resetPose2d(Pose2d pose);
}
