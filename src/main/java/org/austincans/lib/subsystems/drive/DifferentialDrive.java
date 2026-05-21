package org.austincans.lib.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import org.austincans.lib.Constants;
import org.austincans.lib.configs.DifferentialDriveConfig;
import org.austincans.lib.configs.MotorConfig;
import org.austincans.lib.configs.WheelConfig;
import org.austincans.lib.hardware.sensors.gyros.GyroIO;
import org.austincans.lib.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*
 * Links to complete DifferentialDrive
 * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/diff_drive/src/main/java/frc/robot/subsystems/drive/Drive.java
 * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/diff_drive/src/main/java/frc/robot/subsystems/drive/DriveIOSpark.java
 * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/diff_drive/src/main/java/frc/robot/subsystems/drive/DriveIOTalonFX.java
 * https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/diff_drive/src/main/java/frc/robot/subsystems/drive/DriveConstants.java
 */

/**
 * TODO: This is untested
 * TODO: This does not extend AdvancedSubsystem
 */
public class DifferentialDrive<MotorCnfg extends MotorConfig> extends SubsystemBase {
  private final DifferentialDriveIO io;
  private final DifferentialDriveIORecordsAutoLogged records = new DifferentialDriveIORecordsAutoLogged();
  private final DifferentialDriveIORequestsAutoLogged requests = new DifferentialDriveIORequestsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIORecordsAutoLogged gyroRecords = new GyroIORecordsAutoLogged();

  private final double wheelRadiusMeters;
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final SysIdRoutine sysId;
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;

  protected DifferentialDrive(DifferentialDriveIO io, DifferentialDriveConfig<MotorCnfg> driveConfig, DCMotor driveModuleGearbox, double driveModuleCurrentLimitAmps) {
    this.io = io;

    // Configure AutoBuilder for PathPlanner
    WheelConfig wheelConfig = driveConfig.wheelConfig;
    wheelRadiusMeters = wheelConfig.radiusMeters;
    MotorConfig motorConfig = driveConfig.leftLeaderMotorConfig;
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                getLeftVelocityMetersPerSec(),
                getRightVelocityMetersPerSec())),
        (velocities, feedforwards) -> runClosedLoop(velocities),
        new PPLTVController(Constants.loopPeriodSecs, driveConfig.maxDriveSpeedMetersPerSec),
        new RobotConfig(
            driveConfig.robotMassKg,
            driveConfig.robotMomentOfInertiaKgMetersSq,
            new ModuleConfig(
                wheelRadiusMeters,
                driveConfig.maxDriveSpeedMetersPerSec,
                wheelConfig.coefficientOfFriction,
                driveModuleGearbox,
                driveModuleCurrentLimitAmps,
                driveConfig.trackWidthMeters,
                2
            ),
            driveConfig.trackWidthMeters),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runOpenLoop(voltage.in(Volts), voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateRecords(records);
    Logger.processInputs(getName(), records);
    gyroIO.updateInputs(gyroRecords);
    Logger.processInputs(getName() + "/Gyro", gyroRecords);

    // Update gyro angle
    if (gyroRecords.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroRecords.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist =
          kinematics.toTwist2d(
              getLeftPositionMeters() - lastLeftPositionMeters,
              getRightPositionMeters() - lastRightPositionMeters);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      lastLeftPositionMeters = getLeftPositionMeters();
      lastRightPositionMeters = getRightPositionMeters();
    }

    poseEstimator.update(rawGyroRotation, getLeftPosition, null);

    io.applyRequests(requests);
  }

  /** Runs the drive at the desired velocity. */
  public void runClosedLoop(ChassisSpeeds velocities) {
    var wheelVelocities = kinematics.toWheelSpeeds(velocities);

    double leftRadPerSec = wheelVelocities.leftMetersPerSecond / wheelRadiusMeters;
    double rightRadPerSec = wheelVelocities.rightMetersPerSecond / wheelRadiusMeters;
    Logger.recordOutput("Drive/LeftSetpointRadPerSec", leftRadPerSec);
    Logger.recordOutput("Drive/RightSetpointRadPerSec", rightRadPerSec);

    double leftFFVolts = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
    double rightFFVolts = kS * Math.signum(rightRadPerSec) + kV * rightRadPerSec;
    requests.setVelocities(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
  }

  /** Runs the drive in open loop. */
  public void runOpenLoop(double leftVolts, double rightVolts) {
    requests.setVoltage(leftVolts, rightVolts);
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
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp  The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the position of the left wheels in meters. */
  @AutoLogOutput
  public double getLeftPositionMeters() {
    return records.leftLeaderMotorRecords.positionRad * wheelRadiusMeters;
  }

  /** Returns the position of the right wheels in meters. */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return records.rightLeaderMotorRecords.positionRad * wheelRadiusMeters;
  }

  private double getLeftVelocityRadPerSec() {
    return records.leftLeaderMotorRecords.velocityRadPerSec;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return getLeftVelocityRadPerSec() * wheelRadiusMeters;
  }

  private double getRightVelocityRadPerSec() {
    return records.rightLeaderMotorRecords.velocityRadPerSec; 
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return getRightVelocityRadPerSec() * wheelRadiusMeters;
  }

  /** Returns the average velocity in radians/second. */
  public double getCharacterizationVelocity() {
    return (getLeftVelocityRadPerSec() + getRightVelocityRadPerSec()) / 2.0;
  }
}
