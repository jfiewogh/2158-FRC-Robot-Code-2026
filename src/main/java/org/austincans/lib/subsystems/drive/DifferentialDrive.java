package org.austincans.lib.subsystems.drive;

import java.util.function.DoubleSupplier;
import org.austincans.frc2026.Constants;
import org.austincans.lib.configs.DifferentialDriveConfig;
import org.austincans.lib.configs.PIDConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** 
 * Differential drive pose estimation only uses odometry, no vision. 
 */
public class DifferentialDrive {
    private final DifferentialDriveIO io;
    private final DifferentialDriveIORecordsAutoLogged records = new DifferentialDriveIORecordsAutoLogged();
    private final DifferentialDriveIORequestsAutoLogged requests = new DifferentialDriveIORequestsAutoLogged();

    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final SysIdRoutine sysId;
    private Rotation2d rawGyroRotation = Rotation2d.kZero;

    protected DifferentialDrive(DifferentialDriveIO io, DifferentialDriveConfig driveConfig) {
        this.io = io;
        
        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            swerveState::getEstimatedPose,
            this::setPose2d,
            swerveState::getRobotVelocities,
            (velocities, feedforwards) -> runVelocities(velocities),
            new PPLTVController(Constants.loopPeriodSecs, driveConfig.maxDriveSpeedMetersPerSec),
            new RobotConfig(
                driveConfig.robotMassKg, 
                driveConfig.robotMomentOfInertiaKgMetersSq,
                new ModuleConfig(
                    moduleConfig.wheelRadiusMeters(),
                    
                ),
                moduleTranslations),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
    }

    public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
        io.runVelocities(xInput.getAsDouble(), 0, omegaInput.getAsDouble());
    }
}
