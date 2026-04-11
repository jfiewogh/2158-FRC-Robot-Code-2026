// Copyright (c) 2026 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// oooooooooooo ooooooooo.     .oooooo.        ooooooooo.              .o8                     .   
// `888'     `8 `888   `Y88.  d8P'  `Y8b       `888   `Y88.           "888                   .o8   
//  888          888   .d88' 888                888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo 
//  888oooo8     888ooo88P'  888                888ooo88P'  d88' `88b  d88' `88b d88' `88b   888   
//  888    "     888`88b.    888                888`88b.    888   888  888   888 888   888   888   
//  888          888  `88b.  `88b    ooo        888  `88b.  888   888  888   888 888   888   888 . 
// o888o        o888o  o888o  `Y8bood8P'       o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888" 
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2026.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import lombok.Setter;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.RobotState;
import org.austincans.frc2026.RobotState.OdometryObservation;
import org.austincans.frc2026.RobotState.VisionObservation;
import org.austincans.frc2026.generated.TunerConstants;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.vision.Vision;
import org.austincans.frc2026.util.LocalADStarAK;
import org.austincans.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends AdvancedSubsystem implements Vision.VisionConsumer {
  public static double driveSpeedMultiplier = 1;
  public static double driveTestMultiplier = 0.7; // For when in testing mode
  public static double turningMultiplier = 0.95;
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = DriveConstants.robotMassKG;
  private static final double ROBOT_MOI = DriveConstants.robotMOI;
  private static final double WHEEL_COF = DriveConstants.wheelCOF;
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  @AutoLogOutput private boolean velocityMode = false;
  @AutoLogOutput private boolean brakeModeEnabled = true;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
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
  private boolean hasStartedCoast = false;

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    gyroIO.registerSelfCheckHardware(this);

    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;

    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    registerCheckCommand(systemCheckCommand());
    lastMovementTimer.start();
    for (var module : modules) {
      module.brake();
    }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(7.5, 0.0, 2), new PIDConstants(7.5, 0.0, 2)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BRAKE,
    ALWAYS_COAST
  }

  @Setter @AutoLogOutput private CoastRequest coastRequest = CoastRequest.ALWAYS_BRAKE;

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    // LoggedTracer.record("Drive/GyroInputs");
    // Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Log empty setpoint states when disabled
    // if (DriverStation.isDisabled()) {
    // Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
    // Logger.recordOutput("SwerveStates/SetpointsOptimized", new
    // SwerveModuleState[] {});
    // }

    // Update odometry
    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(
                Timer.getTimestamp(),
                getModulePositions(),
                Optional.ofNullable(gyroInputs.connected ? gyroInputs.rollPosition : null),
                Optional.ofNullable(gyroInputs.connected ? gyroInputs.pitchPosition : null),
                Optional.ofNullable(gyroInputs.connected ? gyroInputs.yawPosition : null)));
    RobotState.getInstance().setRobotVelocity(getChassisSpeeds());
    // Update brake mode
    if (Arrays.stream(modules)
        .anyMatch(
            (module) ->
                Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecondThreshold.get())) {
      lastMovementTimer.reset();
    }
    if (DriverStation.isDisabled()) {
      if (hasStartedCoast || lastMovementTimer.hasElapsed(coastWaitTime.get())) {
        for (var module : modules) {
          module.coast();
        }
        hasStartedCoast = true;
      } else {
        for (var module : modules) {
          module.brake();
        }
      }
    } else {
      hasStartedCoast = false;
    }

    RobotState.getInstance().updateField();

    // Record cycle time
    // LoggedTracer.record("Drive/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    for (var module : modules) {
      module.periodicAfterScheduler();
    }
    // LoggedTracer.record("Drive/PeriodicAfterScheduler");
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.maxLinearSpeed);

    // Log unoptimized setpoints and setpoint speeds
    // Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    // Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Publish setpoints
    RobotState.getInstance().setRobotSetpointVelocity(discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
    RobotState.getInstance().setRobotSetpointVelocity(new ChassisSpeeds());
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runAngularCharacterization(double output) {
    velocityMode = false;
    for (int i = 0; i < 4; i++) {
      modules[i].runAngularCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
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
   * Returns a command to run a dynamic test in the specified direction and with counterClockwise
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

  /** Stops the drive and turns the modules to an O arrangement to resist movement. */
  public void stopWithO() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle().plus(Rotation2d.kCW_90deg);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured_Field")
  public ChassisSpeeds getFieldRelVelocities() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the raw gyro rotation read by the IMU */
  public Rotation2d getGyroRotation() {
    return gyroInputs.yawPosition;
  }

  public double getGyroRateDegPerSec() {
    return Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec);
  }

  public Pose3d getPose3d() {
    return new Pose3d(RobotState.getInstance().getEstimatedPose());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return getPose3d().toPose2d();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    RobotState.getInstance().resetPose(pose);
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

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
        * driveSpeedMultiplier
        * driveTestMultiplier;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec()
        * driveSpeedMultiplier
        * driveTestMultiplier
        / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.none();
    // return Commands.sequence(
    // Commands.defer(
    // () ->
    // Commands.sequence(
    // modules[0].getSystemCheckCommand(),
    // modules[1].getSystemCheckCommand(),
    // modules[2].getSystemCheckCommand(),
    // modules[3].getSystemCheckCommand()),
    // Set.of(this)),
    // Commands.waitUntil(
    // () ->
    // modules[0].getCurrentCommand() == null
    // && modules[1].getCurrentCommand() == null
    // && modules[2].getCurrentCommand() == null
    // && modules[3].getCurrentCommand() == null),
    // Commands.runOnce(() -> runVelocity(new ChassisSpeeds(0, 0, 0.5))),
    // Commands.waitSeconds(2.0))
    // .andThen(Commands.runOnce(() -> runVelocity(new ChassisSpeeds()), this));
  }
}
