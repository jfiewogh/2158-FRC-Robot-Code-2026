package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorIO;

public class DifferentialDriveIOReal implements DifferentialDriveIO {
    private final MotorIO leftLeader;
    private final MotorIO leftFollower;
    private final MotorIO rightLeader;
    private final MotorIO rightFollower;

    public DifferentialDriveIOReal(MotorIO leftLeader, MotorIO leftFollower, MotorIO rightLeader, MotorIO rightFollower) {
        this.leftLeader = leftLeader;
        this.leftFollower = leftFollower;
        this.rightLeader = rightLeader;
        this.rightFollower = rightFollower;
    }

    @Override
    public void updateRecords(DifferentialDriveIORecords records) {
        leftLeader.updateRecords(records.leftLeaderMotorRecords);
        leftFollower.updateRecords(records.leftFollowerMotorRecords);
        rightLeader.updateRecords(records.rightLeaderMotorRecords);
        rightFollower.updateRecords(records.rightFollowerMotorRecords);
    }

    @Override
    public void applyRequests(DifferentialDriveIORequests requests) {
        leftLeader.applyRequests(requests.leftLeaderMotorRequests);
        rightLeader.applyRequests(requests.rightLeaderMotorRequests);
    }
}
