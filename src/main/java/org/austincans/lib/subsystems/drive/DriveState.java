package org.austincans.lib.subsystems.drive;

/** Based on the RobotState but only including drive pose estimation */
public class DriveState {
    private static DriveState instance;

    public static DriveState getInstance() {
        if (instance == null) {
            instance = new DriveState();
        }
        return instance;
    }

    private DriveState() {}
}
