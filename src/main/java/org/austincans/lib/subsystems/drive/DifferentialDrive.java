package org.austincans.lib.subsystems.drive;

import java.util.function.DoubleSupplier;
import org.austincans.lib.configs.DifferentialDriveConfig;
import org.austincans.lib.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
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
                                driveConfig.maxDriveSpeedMetersPerSec,
                                driveConfig.wheelCoefficientOfFriction

                        ),
                        moduleTranslations),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            () -> {
                Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
    }

    @Override
    public void periodic() {
        io.updateRecords(inputs);
        gyroIO.updateInputs();
    }

    public void runClosedLoop(ChassisSpeeds velocities) {

    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(
            rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters(), pose);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
        io.runVelocities(xInput.getAsDouble(), 0, omegaInput.getAsDouble());
    }
}
