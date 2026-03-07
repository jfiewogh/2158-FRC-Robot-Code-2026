package org.austincans.lib.subsystems.drive;

import org.austincans.frc2026.subsystems.drive.DriveIOInputsAutoLogged;
import org.austincans.lib.subsystems.drive.DriveIO;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveBase extends AdvancedSubsystem implements Vision.VisionConsumer {
    protected final String name;
    protected final DriveIO io;
    protected final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    protected DriveBase(String name, DriveIO io, PathFollowingController pathFollowingController, RobotConfig robotConfig) {
        this.name = name;
        this.io = io;

        AutoBuilder.configure(
            this::getPose2d,
            this::setPose2d,
            this::getVelocities,
            (velocities, feedforwards) -> runVelocities(velocities),
            pathFollowingController,
            robotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public void runVelocities(ChassisSpeeds velocities) {
        io.runVelocities(velocities);
    }
    public void runVelocities(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        runVelocities(new ChassisSpeeds(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec));
    }

    public Pose3d getPose3d() {
        return inputs.pose;
    }
    
    public Pose2d getPose2d() {
        return getPose3d().toPose2d();
    }

    public void setPose2d(Pose2d pose) {
        return io.resetPose2d(pose);
    }

    public ChassisSpeeds getVelocities() {
        return inputs.velocities;
    }

    @Override
    public void accept(Pose3d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'accept'");
    }

    @Override
    protected Command systemCheckCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
    }
    
}
