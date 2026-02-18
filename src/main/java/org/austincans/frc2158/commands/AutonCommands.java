package org.austincans.frc2158.commands;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.intake.Intake;
import org.austincans.frc2158.subsystems.intake.IntakeConstants;
import org.austincans.frc2158.subsystems.vision.ObjectDetection;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonCommands {
    private static final double maxVelocityMetersPerSec = 5;
    private static final double maxAccelerationMetersPerSecSq = 10;
    private static ProfiledPIDController drivePID = new ProfiledPIDController(1.5,0.0,0.0,new TrapezoidProfile.Constraints(maxVelocityMetersPerSec, maxAccelerationMetersPerSecSq));

    private static final double maxVelocityRadPerSec = Math.PI;
    private static final double maxAccelerationRadPerSecSq = Math.PI * 2;
    private static ProfiledPIDController rotationPID = new ProfiledPIDController(
            1.5,
            0,
            0,
            new TrapezoidProfile.Constraints(maxVelocityRadPerSec, maxAccelerationRadPerSecSq));

    static {
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private static void driveToPose(Drive drive, Pose2d pose) {
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
            drivePID.calculate(drive.getPose().getX(), pose.getX()),
            drivePID.calculate(drive.getPose().getY(), pose.getY()),
            rotationPID.calculate(drive.getPose().getRotation().getRadians(),
            Math.atan2(pose.getY() - drive.getPose().getY(), pose.getX() - drive.getPose().getX())),
            drive.getRotation());
        drive.runVelocity(speed);
    }

    private static void intakeUntilFull(Intake intake) {
        if (intake.getGamePieceAmount() < IntakeConstants.maxIntakeAmount) {
            intake.setVoltage(6);
        } else {
            intake.setVoltage(0);
        }
    }

    public static Command intakeUntilFullCommand(Intake intake) {
        return Commands.run(() -> intakeUntilFull(intake));
    }

    public static Command driveToPoseCommand(Drive drive, ObjectDetection objectDetection) {
        Supplier<Pose3d> nearestGamePiecePoseSupplier = objectDetection::getNearestGamePiecePose;
        return Commands.run(() -> {
            if (nearestGamePiecePoseSupplier.get().toPose2d() != null){
                driveToPose(drive, nearestGamePiecePoseSupplier.get().toPose2d());
            }
        });
    }

    public static Command alignDriveIntakeCommand(ObjectDetection objectDetection, Drive drive, Intake intake) {
        return Commands.sequence(
            driveToPoseCommand(drive, objectDetection),
            intakeUntilFullCommand(intake));
    }
}
