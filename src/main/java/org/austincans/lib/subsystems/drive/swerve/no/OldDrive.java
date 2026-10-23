package org.austincans.lib.subsystems.drive.swerve.no;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.austincans.frc2026.util.swerve.SwerveSetpoint;
import org.austincans.lib.Constants;
import org.austincans.lib.io.sensors.gyros.GyroIO;
import org.austincans.lib.io.sensors.gyros.inputs.GyroIOInputs;
import org.austincans.lib.subsystems.drive.swerve.SwerveConfig;
import org.austincans.lib.subsystems.vision.Vision;
import org.austincans.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/subsystems/drive/Drive.java

public class OldDrive implements Vision.VisionConsumer {
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs;

  private final SysIdRoutine sysId;
  private final Debouncer gyroConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private static final LoggedTunableNumber coastWaitTime = new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecondThreshold = new LoggedTunableNumber(
      "Drive/CoastMetersPerSecThreshold", .05);

  private final Timer lastMovementTimer = new Timer();

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  private final Translation2d[] moduleTranslations;
  private final SwerveDriveKinematics kinematics;
  private final SwerveSetpointGenerator swerveSetpointGenerator;
  private final ModuleLimits moduleLimitsFree;
  private final double wheelbaseMeters;
  private final double trackWidthMeters;

  private RobotState swerveState;

  private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(),
      new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
      });

  public Swerve(ModuleIO frontLeft, ModuleIO frontRight, ModuleIO backLeft, ModuleIO backRight, GyroIO gyroIO, SwerveConfig swerveConfig) {
        this.gyroIO = gyroIO;

        moduleLimitsFree = moduleConfig.moduleLimitsFree();
        wheelbaseMeters = swerveConfig.wheelbaseMeters;
        trackWidthMeters = swerveConfig.trackWidthMeters;
        moduleTranslations = new Translation2d[] {
            new Translation2d(wheelbaseMeters / 2, trackWidthMeters / 2),
            new Translation2d(wheelbaseMeters / 2, -trackWidthMeters / 2),
            new Translation2d(-wheelbaseMeters / 2, trackWidthMeters / 2),
            new Translation2d(-wheelbaseMeters / 2, -trackWidthMeters / 2)};
        kinematics = new SwerveDriveKinematics(moduleTranslations);
        swerveSetpointGenerator = new SwerveSetpointGenerator(kinematics);

        // Configure AutoBuilder for PathPlanner
        PIDConfig drivePIDConfig = swerveConfig.drivePIDConfig;
        PIDConfig steerPIDConfig = swerveConfig.steerPIDConfig;
        AutoBuilder.configure(
            swerveState::getEstimatedPose,
            this::setPose2d,
            swerveState::getRobotVelocities,
            (velocities, feedforwards) -> runVelocities(velocities),
            new PPHolonomicDriveController(
                new PIDConstants(drivePIDConfig.kP(), drivePIDConfig.kI(), drivePIDConfig.kD()),
                new PIDConstants(steerPIDConfig.kP(), steerPIDConfig.kI(), steerPIDConfig.kD())),
            new RobotConfig(
                swerveConfig.robotMassKg, 
                swerveConfig.robotMomentOfInertiaKgMetersSq,
                new ModuleConfig(
                    moduleConfig.wheelRadiusMeters(),
                    
                ),
                moduleTranslations),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);

        sysId = new SysIdRoutine(null, null);

        gyroIO.registerSelfCheckHardware(this);
    }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroRecords);
    Logger.processInputs("Drive/Gyro", gyroRecords);

    for (var module : modules) {
      module.periodic();
    }

    // Send odometry updates to robot state
    swerveState.addOdometryObservation(
        new SwerveState.OdometryObservation(
            Timer.getTimestamp(),
            getModulePositions(),
            Optional.ofNullable(gyroRecords.connected ? gyroRecords.rollPosition : null),
            Optional.ofNullable(gyroRecords.connected ? gyroRecords.pitchPosition : null),
            Optional.ofNullable(gyroRecords.connected ? gyroRecords.yawPosition : null)));

    // Log 3D robot pose
    Logger.recordOutput(
        "RobotState/EstimatedPose3d",
        new Pose3d(swerveState.getEstimatedPose())
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroRecords.pitchPosition.getRadians())
                        * wheelbaseMeters
                        / 2.0,
                    0.0,
                    gyroRecords.pitchPosition.getRadians(),
                    0.0))
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroRecords.rollPosition.getRadians())
                        * trackWidthMeters
                        / 2.0,
                    gyroRecords.rollPosition.getRadians(),
                    0.0,
                    0.0)));
    swerveState.setRobotVelocities(getVelocities());
  }

  /**
   * Runs the drive at the desired velocity.
   * 
   * @param speeds Speeds in meters/sec
   */
  public void runVelocities(ChassisSpeeds velocities) {
    velocityMode = true;

    // Calculate module setpoints
    ChassisSpeeds discreteVelocities = ChassisSpeeds.discretize(velocities, Constants.loopPeriodSec);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteVelocities);
    currentSetpoint = swerveSetpointGenerator.generateSetpoint(
        moduleLimitsFree,
        currentSetpoint,
        discreteVelocities,
        Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules.get(i).runSetpoint(setpointStates[i]);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    velocityMode = false;
    for (var module : modules) {
      module.runCharacterization(output);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runAngularCharacterization(double output) {
    velocityMode = false;
    for (int i = 0; i < 4; i++) {
      modules.get(i).runAngularCharacterization(output, moduleTranslations[i]);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocities(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Returns a command to run a dynamic test in the specified direction and with
   * counterClockwise
   * Rotation.
   */
  public Command sysIdDynamicAngular(SysIdRoutine.Direction direction) {
    return run(() -> runAngularCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistaticAngular(SysIdRoutine.Direction direction) {
    return run(() -> runAngularCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Used for odometry */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules.get(i).getPosition();
    }
    return positions;
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules.get(i).getState();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getVelocities() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void setPose2d(Pose2d pose) {
    swerveState.resetPose(pose);
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose3d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveState
        .addVisionObservation(
            new SwerveState.VisionObservation(
                timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs));
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
        Commands.defer(
            () -> Commands.sequence(
                modules.get(0).getSystemCheckCommand(),
                modules.get(1).getSystemCheckCommand(),
                modules.get(2).getSystemCheckCommand(),
                modules.get(3).getSystemCheckCommand()),
            Set.of(this)),
        Commands.waitUntil(
            () -> modules.get(0).getCurrentCommand() == null
                && modules.get(1).getCurrentCommand() == null
                && modules.get(2).getCurrentCommand() == null
                && modules.get(3).getCurrentCommand() == null),
        Commands.runOnce(() -> runVelocities(new ChassisSpeeds(0, 0, 0.5))),
        Commands.waitSeconds(2.0))
        .andThen(Commands.runOnce(() -> runVelocities(new ChassisSpeeds()), this));
  }
}
