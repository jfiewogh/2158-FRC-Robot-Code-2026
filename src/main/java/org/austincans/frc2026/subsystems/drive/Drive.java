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
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
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
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Setter;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.Constants.Mode;
import org.austincans.frc2026.Robot;
import org.austincans.frc2026.RobotState;
import org.austincans.frc2026.RobotState.OdometryObservation;
import org.austincans.frc2026.RobotState.VisionObservation;
import org.austincans.frc2026.generated.TunerConstants;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.vision.Vision;
import org.austincans.frc2026.util.LocalADStarAK;
import org.austincans.frc2026.util.LoggedTunableNumber;
import org.austincans.frc2026.util.swerve.SwerveSetpoint;
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

  private final DriveIO driveIO;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
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

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  public static boolean joystickHasInput = false;

  public static boolean isDriving = false;

  public boolean isDriving() {
    return joystickHasInput;
  }

  public Drive(
      DriveIO driveIO,
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.driveIO = driveIO;
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    driveIO.setModules(modules);
    registerCheckCommand(systemCheckCommand());
    lastMovementTimer.start();
    setBrakeMode(true);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getRobotRelVelocities,
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

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @Setter @AutoLogOutput private CoastRequest coastRequest = CoastRequest.ALWAYS_BRAKE;

  @Override
  public void periodic() {
    driveIO.updateInputs(driveInputs);
    Logger.processInputs("Drive/Inputs", driveInputs);

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    gyroIO.checkForFaults().forEach(this::addHardwareFault);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Send odometry updates to robot state
    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(
                Timer.getTimestamp(),
                getModulePositions(),
                Optional.ofNullable(gyroInputs.connected ? gyroInputs.rollPosition : null),
                Optional.ofNullable(gyroInputs.connected ? gyroInputs.pitchPosition : null),
                Optional.ofNullable(gyroInputs.connected ? gyroInputs.yawPosition : null)));

    // Log 3D robot pose
    Logger.recordOutput(
        "RobotState/EstimatedPose3d", new Pose3d(RobotState.getInstance().getEstimatedPose()));
    
    RobotState.getInstance().setRobotVelocity(getRobotRelVelocities());
    // Update brake mode
    // Reset movement timer if velocity above threshold
    if (Arrays.stream(modules)
        .anyMatch(
            (module) ->
                Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecondThreshold.get())) {
      lastMovementTimer.reset();
    }

    if (DriverStation.isEnabled()) {
      coastRequest = CoastRequest.ALWAYS_BRAKE;
    }

    switch (coastRequest) {
      case AUTOMATIC -> {
        if (DriverStation.isEnabled()) {
          setBrakeMode(true);
        } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
          setBrakeMode(false);
        }
      }
      case ALWAYS_BRAKE -> {
        setBrakeMode(true);
      }
      case ALWAYS_COAST -> {
        setBrakeMode(false);
      }
    }

    // Update current setpoint if not in velocity mode
    if (!velocityMode) {
      driveIO.updateSetpoint(new SwerveSetpoint(getRobotRelVelocities(), getModuleStates()));
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(
        !gyroConnectedDebouncer.calculate(gyroInputs.connected)
            && Constants.getMode() != Mode.SIM
            && !Robot.isJITing());
    isDriving = isDriving();
  }

  /** Set brake mode to {@code enabled} doesn't change brake mode if already set. */
  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    velocityMode = true;
    driveIO.runVelocity(speeds);
  }

  /**
   * Runs the drive at the desired velocity with setpoint module forces.
   *
   * @param speeds Speeds in meters/sec
   * @param moduleForces The forces applied to each module
   */
  public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {
    driveIO.runVelocity(speeds, moduleForces);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    velocityMode = false;
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
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
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    driveIO.resetHeadings(headings);
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

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getRobotRelVelocities() {
    return driveInputs.robotRelVelocities;
  }

  public ChassisSpeeds getFieldRelVelocities() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelVelocities(), getRotation());
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
    return driveInputs.pose;
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
    driveIO.resetPose(pose);
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
