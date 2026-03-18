package org.austincans.lib.subsystems.drive;

import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.austincans.frc2026.RobotState;
import org.austincans.frc2026.RobotState.VisionObservation;
import org.austincans.lib.configs.DriveConfigSwerve;
import org.austincans.lib.configs.GyroConfigPigeon2;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.subsystems.vision.Vision;
import org.austincans.lib.util.LoggedTunableNumber;
import org.austincans.lib.util.swerve.ModuleLimits;
import org.austincans.lib.util.swerve.SwerveSetpoint;
import org.austincans.lib.util.swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.AutoLogOutput;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveDrive extends AdvancedSubsystem implements Vision.VisionConsumer {
    @AutoLogOutput private boolean velocityMode = false;
    @AutoLogOutput private boolean brakeModeEnabled = true;

    private final SwerveDriveIO io;
    private final SwerveDriveIORecordsAutoLogged records = new SwerveDriveIORecordsAutoLogged();
    private final SwerveDriveIORequestsAutoLogged requests = new SwerveDriveIORequestsAutoLogged();
    private final Lock odometryLock = new ReentrantLock();
    private final Gyro gyro;
    private final Module[] modules;
    private final SysIdRoutine sysId;
    private final Debouncer gyroConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
    
    private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
    private static final LoggedTunableNumber coastMetersPerSecondThreshold =
      new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", .05);

    private final Timer lastMovementTimer = new Timer();

    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    private final Translation2d[] moduleTranslations;
    private final SwerveDriveKinematics kinematics;
    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private final ModuleLimits moduleLimitsFree;

    private final SwerveDriveState swerveDriveState;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(),
        new SwerveModuleState[] {
            new SwerveModuleState(), 
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        }
    );

    public SwerveDrive(DriveConfigSwerve driveConfig, PPHolonomicDriveController pathFollowingController, RobotConfig robotConfig) {
        
    }

    @Override
    public void periodic() {
        driveIO.updateInputs(driveInputs);
        

        for (Module module : modules) {
            module.periodic();
        }
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(
        Pose3d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        RobotState.getInstance()
            .addVisionObservation(
                new VisionObservation(
                    timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs));
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.sequence(
                Commands.defer(
                    () ->
                        Commands.sequence(
                            modules[0].getSystemCheckCommand(),
                            modules[1].getSystemCheckCommand(),
                            modules[2].getSystemCheckCommand(),
                            modules[3].getSystemCheckCommand()),
                    Set.of(this)),
                Commands.waitUntil(
                    () ->
                        modules[0].getCurrentCommand() == null
                            && modules[1].getCurrentCommand() == null
                            && modules[2].getCurrentCommand() == null
                            && modules[3].getCurrentCommand() == null),
                Commands.runOnce(() -> runVelocity(new ChassisSpeeds(0, 0, 0.5))),
                Commands.waitSeconds(2.0))
            .andThen(Commands.runOnce(() -> runVelocity(new ChassisSpeeds()), this));
    }
}
