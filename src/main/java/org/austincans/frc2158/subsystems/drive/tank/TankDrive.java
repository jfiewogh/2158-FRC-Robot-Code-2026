package org.austincans.frc2158.subsystems.drive.tank;

import java.util.function.DoubleSupplier;

import org.austincans.frc2158.subsystems.drive.DriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class TankDrive {
    private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
        DriveConstants.trackWidthY);

    private PWMMotorController[] motors = new PWMMotorController[] {
        new PWMMotorController("frontLeft", 0) {},
        new PWMMotorController("frontRight", 1) {},
        new PWMMotorController("backLeft", 2) {},
        new PWMMotorController("backRight", 3) {},
    };
    
    // private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    //     Rotation2d.kZero, motors[0].
    // );

    public TankDrive() {
        // RobotConfig config;
        // try{
        //     config = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        //     // Handle exception as needed
        //     e.printStackTrace();
        // }

        // AutoBuilder.configure(
        //     this::getPose, // Robot pose supplier
        //     this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //     (speeds, feedforwards) -> runChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        //     new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
        //     config, // The robot configuration
        //     () -> {
        //       // Boolean supplier that controls when the path will be mirrored for the red alliance
        //       // This will flip the path being followed to the red side of the field.
        //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //       var alliance = DriverStation.getAlliance();
        //       if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //       }
        //       return false;
        //     },
        //     this // Reference to this subsystem to set requirements
        // );
    }

    public void runChassisSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        double leftVelocity = Math.min(wheelSpeeds.leftMetersPerSecond / DriveConstants.maxLinearSpeed, DriveConstants.maxLinearSpeed);
        double rightVelocity = Math.min(wheelSpeeds.rightMetersPerSecond / DriveConstants.maxLinearSpeed, DriveConstants.maxLinearSpeed);
        motors[0].set(leftVelocity);
        motors[1].set(leftVelocity);
        motors[2].set(rightVelocity);
        motors[3].set(rightVelocity);
    }

    public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
        runChassisSpeeds(new ChassisSpeeds(xInput.getAsDouble(), 0, omegaInput.getAsDouble()));
    }

    // public Pose2d getPose() {
    //     return 
    // }

    // public Pose2d resetPose() {

    // }

    // public ChassisSpeeds getRobotRelativeSpeeds() {
    //     return 
    // }
}
