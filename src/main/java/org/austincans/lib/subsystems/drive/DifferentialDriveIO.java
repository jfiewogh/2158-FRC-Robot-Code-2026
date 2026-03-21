package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorIO.MotorIORecords;
import org.austincans.lib.motors.MotorIO.MotorIORequests;
import org.littletonrobotics.junction.AutoLog;

public interface DifferentialDriveIO {
    @AutoLog
    public class DifferentialDriveIORecords {
        public MotorIORecords frontLeftMotorRecords = new MotorIORecords();
        public MotorIORecords frontRightMotorRecords = new MotorIORecords();
        public MotorIORecords backLeftMotorRecords = new MotorIORecords();
        public MotorIORecords backRightMotorRecords = new MotorIORecords();
    }

    @AutoLog
    public class DifferentialDriveIORequests {
        public MotorIORequests frontLeftMotorRequests = new MotorIORequests();
        public MotorIORequests frontRightMotorRequests = new MotorIORequests();
        public MotorIORequests backLeftMotorRequests = new MotorIORequests();
        public MotorIORequests backRightMotorRequests = new MotorIORequests();
    }

    public void updateRecords(DifferentialDriveIORecords records);

    public void applyRequests(DifferentialDriveIORequests requests);
}
