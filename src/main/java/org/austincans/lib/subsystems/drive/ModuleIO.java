package org.austincans.lib.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveTorqueCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;
        public double turnStatorCurrentAmps = 0.0;
        public double turnTorqueCurrentAmps = 0.0; 
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void setDriveVelocity(double velocityRadPerSec);

    public void setSteerPosition(Rotation2d rotation);
}
