package org.austincans.frc2158.commands;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Classifier;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.RawDetection;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.austincans.frc2158.RobotState;
import org.austincans.frc2158.math.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2158.subsystems.vision.ObjectDetection;



public class LimelightHelpersCommands {
    private final ObjectDetection nearestGamePiece;

    final double Kp = 0;
    final double Ki = 0;
    final double Kd = 0;
    final double maxVelocity = 5;
    final double maxAcceleration = 10;
    static LimelightTarget_Classifier[] detections = LimelightHelpers.getLatestResults("").targets_Classifier;
    ProfiledPIDController pid = new ProfiledPIDController(Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    public LimelightHelpersCommands(ObjectDetection nearestGamePiece){
        this.nearestGamePiece = nearestGamePiece;
    }

    public void alignRobotToAngle(Drive drive, Rotation2d goalDegrees){
        drive.runVelocity(new ChassisSpeeds(0,
         0,
          pid.calculate(RobotState.getInstance().getRotation().getDegrees(), goalDegrees.getDegrees())));
    }

    public Command alignHeadingCommand(Drive drive){
        return Commands.run(() -> alignRobotToAngle(drive, 
        nearestGamePiece.getNearestPose().toPose2d().getRotation()));
    }
}