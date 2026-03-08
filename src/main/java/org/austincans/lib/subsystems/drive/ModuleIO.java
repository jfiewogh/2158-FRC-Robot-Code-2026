package org.austincans.lib.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        public MotorIOInputsAutoLogged driveMotorInputs = new MotorIOInputsAutoLogged();
        public MotorIOInputsAutoLogged steerMotorInputs = new MotorIOInputsAutoLogged();
        public AbsoluteEncoderIOInputsAutoLogged steerAbsoluteEncoderInputs = new AbsoluteEncoderIOInputsAutoLogged();
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void runSetpoint(SwerveModuleState state);

    public SwerveModulePosition getPosition();

    public SwerveModuleState getState();
}
