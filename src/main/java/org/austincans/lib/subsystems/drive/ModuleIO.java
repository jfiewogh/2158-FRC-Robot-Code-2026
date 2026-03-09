package org.austincans.lib.subsystems.drive;

import org.austincans.lib.encoders.AbsoluteEncoderIO.AbsoluteEncoderIOInputs;
import org.austincans.lib.motors.MotorIO.MotorIOInputs;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        public MotorIOInputs driveMotorInputs;
        public MotorIOInputs steerMotorInputs;
        public AbsoluteEncoderIOInputs steerAbsoluteEncoderInputs;
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void setDriveOpenLoop(double output);

    public void setSteerOpenLoop(double output);

    public void setDriveVelocity(double velocityRadPerSec);

    public void setSteerAngle(Rotation2d angle);

    public void stop();
}
