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
//
// Updated on Saturday January 17, 2026 at 17:14:59 CST

package org.austincans.frc2158.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
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
import edu.wpi.first.math.geometry.Twist3d;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import lombok.Setter;
import org.austincans.frc2158.Constants;
import org.austincans.frc2158.Constants.Mode;
import org.austincans.frc2158.Robot;
import org.austincans.frc2158.RobotState;
import org.austincans.frc2158.RobotState.VisionObservation;
import org.austincans.frc2158.generated.TunerConstants;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2158.subsystems.vision.Vision;
import org.austincans.frc2158.util.LocalADStarAK;
import org.austincans.frc2158.util.LoggedTunableNumber;
import org.austincans.frc2158.util.swerve.SwerveSetpoint;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends AdvancedSubsystem implements Vision.VisionConsumer {
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
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
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

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
          .withCustomModuleTranslations(getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getFalcon500(1),
                  TunerConstants.FrontLeft.DriveMotorGearRatio,
                  TunerConstants.FrontLeft.SteerMotorGearRatio,
                  Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                  Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                  Meters.of(TunerConstants.FrontLeft.WheelRadius),
                  KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                  WHEEL_COF));

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

  public static boolean joystickHasInput = true;

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
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
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
    driveIO.updateInputs(driveInputs);
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    gyroIO.checkForFaults().forEach(this::addFault);
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
    double[] sampleTimestamps =
        Constants.getMode() == Mode.SIM
            ? new double[] {Timer.getTimestamp()}
            : gyroInputs.odometryYawTimestamps; // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = modules[j].getOdometryPositions()[i];
      }
      RobotState.getInstance()
          .addOdometryObservation(
              new RobotState.OdometryObservation(
                  wheelPositions,
                  Optional.ofNullable(
                      gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null),
                  sampleTimestamps[i]));

      // Log 3D robot pose
      Logger.recordOutput(
          "RobotState/EstimatedPose3d",
          new Pose3d(RobotState.getInstance().getEstimatedPose())
              .exp(
                  new Twist3d(
                      0.0,
                      0.0,
                      Math.abs(gyroInputs.pitchPosition.getRadians())
                          * DriveConstants.trackWidthX
                          / 2.0,
                      0.0,
                      gyroInputs.pitchPosition.getRadians(),
                      0.0))
              .exp(
                  new Twist3d(
                      0.0,
                      0.0,
                      Math.abs(gyroInputs.rollPosition.getRadians())
                          * DriveConstants.trackWidthY
                          / 2.0,
                      gyroInputs.rollPosition.getRadians(),
                      0.0,
                      0.0)));
    }

    RobotState.getInstance().addDriveSpeeds(getChassisSpeeds());
    RobotState.getInstance().setPitch(gyroInputs.pitchPosition);
    RobotState.getInstance().setRoll(gyroInputs.rollPosition);

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
      driveIO.updateSetpoint(new SwerveSetpoint(getChassisSpeeds(), getModuleStates()));
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(
        !gyroConnectedDebouncer.calculate(gyroInputs.connected)
            && Constants.getMode() != Mode.SIM
            && !Robot.isJITing());
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
  public ChassisSpeeds getChassisSpeeds() {
    return driveInputs.chassisSpeeds;
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

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return driveInputs.pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    driveIO.resetPose(pose);
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    RobotState.getInstance()
        .addVisionObservation(
            new VisionObservation(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs));
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
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
            Commands.runOnce(
                () -> {
                  // TODO: seems like there is a better way to do this
                  CommandScheduler.getInstance().schedule(modules[0].getSystemCheckCommand());
                  CommandScheduler.getInstance().schedule(modules[1].getSystemCheckCommand());
                  CommandScheduler.getInstance().schedule(modules[2].getSystemCheckCommand());
                  CommandScheduler.getInstance().schedule(modules[3].getSystemCheckCommand());
                },
                this),
            // Hack to run module system checks since modules[] does not exist when this method is
            // called
            Commands.waitUntil(
                () ->
                    modules[0].getCurrentCommand() == null
                        && modules[1].getCurrentCommand() == null
                        && modules[2].getCurrentCommand() == null
                        && modules[3].getCurrentCommand() == null),
            Commands.runOnce(() -> runVelocity(new ChassisSpeeds(0, 0, 0.5))),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  runVelocity(new ChassisSpeeds());
                  if (-gyroInputs.imuRate < Units.radiansToDegrees(0.3)) {
                    addFault("[System Check] IMU rate too low", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> runVelocity(new ChassisSpeeds(0, 0, -0.5))),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  runVelocity(new ChassisSpeeds());
                  if (-gyroInputs.imuRate > Units.radiansToDegrees(-0.3)) {
                    addFault("[System Check] IMU rate too low", false, true);
                  }
                },
                this))
        .until(
            () ->
                getFaults().size() > 0
                    || modules[0].getFaults().size() > 0
                    || modules[1].getFaults().size() > 0
                    || modules[2].getFaults().size() > 0
                    || modules[3].getFaults().size() > 0)
        .andThen(Commands.runOnce(() -> runVelocity(new ChassisSpeeds()), this));
  }
}
