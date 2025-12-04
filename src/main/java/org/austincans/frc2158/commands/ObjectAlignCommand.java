package org.austincans.frc2158.commands;

import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightResults;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Classifier;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ObjectAlignCommand {
    static LimelightTarget_Classifier[] results = LimelightHelpers.getLatestResults("").targets_Classifier;
    static LimelightTarget_Classifier closestObject = null;

    public static void alignToClosestObject(Drive drive) {
        if (results.length == 0){
            return;
        } else {
            closestObject = results[0]; // Use Closest Contour sorting
        }

        double kP_Angular = .035;
        double targetAngularVelocity = closestObject.tx * kP_Angular;

        // convert to radians per second for our drive method
        targetAngularVelocity *= 0; // TODO: Replace with swerve max angular speed
        targetAngularVelocity *= -1.0;

        double kP_Lateral = .1;
        double targetLateralVelocity = closestObject.ty * kP_Lateral;
        targetLateralVelocity *= 0; // TODO: Replace with swerve max lateral speed
        targetLateralVelocity *= -1.0;

        drive.runVelocity(new ChassisSpeeds(
                targetLateralVelocity, // Forwards TODO: Replace with Driver Joystick?
                0, // Sideways TODO: Replace with Driver Joystick?
                targetAngularVelocity // Angular
        ));
    }
}
