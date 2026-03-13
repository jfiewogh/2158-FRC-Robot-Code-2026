package org.austincans.lib.subsystems.drive;

import java.util.Set;

import org.austincans.lib.configs.DriveConfigSwerve;
import org.austincans.lib.configs.GyroConfigPigeon2;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.subsystems.vision.Vision;
import org.austincans.lib.util.swerve.ModuleLimits;
import org.austincans.lib.util.swerve.SwerveSetpoint;
import org.austincans.lib.util.swerve.SwerveSetpointGenerator;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SwerveDrive extends AdvancedSubsystem implements Vision.VisionConsumer {
    private final Translation2d[] moduleTranslations;
    private final SwerveDriveKinematics kinematics;
    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private final ModuleLimits moduleLimitsFree;

    private final Module[] modules;
    private final Gyro gyro;

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
    private boolean velocityMode = false;

    public SwerveDrive(DriveConfigSwerve driveConfig, PPHolonomicDriveController pathFollowingController, RobotConfig robotConfig) {
        
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
