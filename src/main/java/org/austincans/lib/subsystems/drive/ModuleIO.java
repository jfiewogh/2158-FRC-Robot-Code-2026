package org.austincans.lib.subsystems.drive;

import org.austincans.lib.encoders.AbsoluteEncoderIO.AbsoluteEncoderIOInputs;
import org.austincans.lib.motors.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        public MotorIOInputs driveMotorInputs;
        public MotorIOInputs steerMotorInputs;
        public AbsoluteEncoderIOInputs steerAbsoluteEncoderInputs ;
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void runSetpoint(SwerveModuleState state);

    public SwerveModulePosition getPosition();

    public SwerveModuleState getState();
}
