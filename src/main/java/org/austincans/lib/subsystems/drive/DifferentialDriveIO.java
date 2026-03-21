package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorIO.MotorIORecords;
import org.austincans.lib.motors.MotorIO.MotorIORequests;
import org.littletonrobotics.junction.AutoLog;

public interface DifferentialDriveIO {
    @AutoLog
    public class DifferentialDriveIORecords {
        public MotorIORecords leftLeaderMotorRecords = new MotorIORecords();
        public MotorIORecords leftFollowerMotorRecords = new MotorIORecords();
        public MotorIORecords rightLeaderMotorRecords = new MotorIORecords();
        public MotorIORecords rightFollowerMotorRecords = new MotorIORecords();
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
