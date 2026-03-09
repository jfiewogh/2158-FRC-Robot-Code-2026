package org.austincans.lib.subsystems.drive;

import org.austincans.lib.subsystems.drive.GyroIO.GyroIOInputs;
import org.austincans.lib.subsystems.drive.ModuleIO.ModuleIOInputs;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** All Drive IOs should have a gyro. If you want a Drive IO with no gyro, you can use GyroIOFake or make a different DriveIO. */
public interface DriveIO {
    @AutoLog
    public class DriveIOInputs {
        public ModuleIOInputs frontLeftInputs;
        public ModuleIOInputs frontRightInputs;
        public ModuleIOInputs backLeftInputs;
        public ModuleIOInputs backRightInputs;
        public GyroIOInputs gyroInputs;
        public Pose3d pose;
        public ChassisSpeeds velocities; // either field or robot relative
    }

    public void updateInputs(DriveIOInputs inputs);

    public void runVelocities(ChassisSpeeds velocities);

    public void resetPose2d(Pose2d pose);
}
