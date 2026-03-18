package org.austincans.lib.subsystems.drive;

import org.austincans.lib.encoders.AbsoluteEncoderIO.AbsoluteEncoderIOInputs;
import org.austincans.lib.motors.MotorIO.MotorIORecords;
import org.austincans.lib.motors.MotorIO.MotorIORequests;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract interface ModuleIO {
    @AutoLog
    public class ModuleIORecords {
        public MotorIORecords driveMotorRecords;
        public MotorIORecords steerMotorRecords;
        public AbsoluteEncoderIOInputs steerAbsoluteEncoderRecords;
    }

    @AutoLog
    public class ModuleIORequests {
        public MotorIORequests driveMotorRequests = new MotorIORequests();
        public MotorIORequests steerMotorRequests = new MotorIORequests();

        public void setDriveOpenLoop(double output) {
            driveMotorRequests.setVoltage(output);
        }
        public void setSteerOpenLoop(double output) {
            steerMotorRequests.setVoltage(output);
        }
        public void setDriveVelocity(double velocityRadPerSec) {
            driveMotorRequests.setVelocity(velocityRadPerSec);
        }
        public void setSteerAngle(Rotation2d angle) {
            steerMotorRequests.setPosition(angle.getRadians());
        }
    }

    public void updateRecords(ModuleIORecords inputs);

    public void applyRequests(ModuleIORequests requests);
}
