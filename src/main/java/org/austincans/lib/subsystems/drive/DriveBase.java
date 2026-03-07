package org.austincans.lib.subsystems.drive;

import org.austincans.lib.subsystems.drive.old.DriveIO;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.subsystems.vision.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveBase extends AdvancedSubsystem implements Vision.VisionConsumer {

    public DriveBase(DriveIO driveIO) {

    }

    @Override
    public void accept(Pose3d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'accept'");
    }

    @Override
    protected Command systemCheckCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
    }
    
}
