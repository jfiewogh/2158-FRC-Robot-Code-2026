package org.austincans.lib.subsystems.drive;

import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.littletonrobotics.junction.AutoLog;

public interface DifferentialDriveIO {
    @AutoLog
    public class DifferentialDriveIOInputs {
        public MotorIOInputs leftLeaderMotorRecords = new MotorIORecords();
        public MotorIOInputs leftFollowerMotorRecords = new MotorIORecords();
        public MotorIOInputs rightLeaderMotorRecords = new MotorIORecords();
        public MotorIOInputs rightFollowerMotorRecords = new MotorIORecords();
    }

    @AutoLog
    public class DifferentialDriveIORequests {
        public MotorIORequests leftLeaderMotorRequests = new MotorIORequests();
        public MotorIORequests rightLeaderMotorRequests = new MotorIORequests();

        public void setVelocities(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
            leftLeaderMotorRequests.
        }
        public void setVoltages(double leftVolts, double rightVolts) {
            leftLeaderMotorRequests.setVoltage(leftVolts);
            rightLeaderMotorRequests.setVoltage(rightVolts);
        }
    }

    public void updateRecords(DifferentialDriveIORecords records);

    public void applyRequests(DifferentialDriveIORequests requests);
}
