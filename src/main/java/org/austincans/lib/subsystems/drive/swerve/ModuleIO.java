package org.austincans.lib.subsystems.drive.swerve;

import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.io.sensors.encoders.inputs.AbsoluteEncoderIOInputs;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        public MotorIOInputs driveMotorInputs;
        public MotorIOInputs steerMotorInputs;
        public AbsoluteEncoderIOInputs steerAbsoluteEncoderInputs;
    }

    @AutoLog
    public class ModuleIOOutputs {
        public MotorIOOutputs driveMotorOutputs = new MotorIOOutputs();
        public MotorIOOutputs steerMotorOutputs = new MotorIOOutputs();

        public void setDriveOpenLoop(Voltage output) {
            driveMotorOutputs.setVoltage(output);
        }
        public void setSteerOpenLoop(Voltage output) {
            steerMotorOutputs.setVoltage(output);
        }
        public void setDriveVelocity(AngularVelocity velocity) {
            driveMotorOutputs.setVelocity(velocity);
        }
        public void setSteerAngle(Rotation2d angle) {
            steerMotorOutputs.setPosition(angle);
        }
    }

    public void updateRecords(ModuleIORecords inputs);

    public void applyRequests(ModuleIORequests requests);
}
