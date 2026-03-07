package org.austincans.lib.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        public boolean driveConnected = false;
        public MotorIOInputsAutoLogged driveMotorInputs;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public MotorIOInputsAutoLogged steerMotorInputs;
        public EncoderIOInputsAutoLogged steerEncoderInputs;
        public Rotation2d steerAbsolutePosition = Rotation2d.kZero;
        public Rotation2d steerRelativePosition = Rotation2d.kZero;
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void runSetpoint(SwerveModuleState state);

    public SwerveModulePosition getPosition();

    public SwerveModuleState getState();
}
