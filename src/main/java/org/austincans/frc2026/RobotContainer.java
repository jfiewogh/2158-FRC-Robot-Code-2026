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

package org.austincans.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.commands.AutonCommands;
import org.austincans.frc2026.commands.DriveCommands;
import org.austincans.frc2026.commands.LimelightHelpersCommands;
import org.austincans.frc2026.commands.TurretCommands;
import org.austincans.frc2026.generated.TunerConstants;
import org.austincans.frc2026.simulation.RobotSimulation3D;
import org.austincans.frc2026.subsystems.drive.*;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.flywheel.FlywheelConstants;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIOSim3D;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIOSparkFlex;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.hood.HoodIOSparkFlex;
import org.austincans.frc2026.subsystems.hood.HoodIOTest;
import org.austincans.frc2026.subsystems.intake.Intake;
import org.austincans.frc2026.subsystems.intake.IntakeIOSim3D;
import org.austincans.frc2026.subsystems.intake.IntakeIOSparkFlex;
import org.austincans.frc2026.subsystems.intake.IntakeIOTalonFXS;
import org.austincans.frc2026.subsystems.led.Led;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.vision.ObjectDetection;
import org.austincans.frc2026.subsystems.vision.Vision;
import org.austincans.frc2026.superstructure.*;
import org.austincans.frc2026.util.HubShiftUtil;
import org.austincans.frc2026.util.LoggedTunableNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystem Enabled // only matters for real robot
  // Maybe move this somewhere else (e.g., RobotConstants)
  public static final boolean driveEnabled = true;
  public static final boolean visionEnabled = true;
  public static final boolean objectDetectionEnabled = false;
  public static final boolean turretEnabled = true;
  public static final boolean hoodEnabled = true;
  public static final boolean rollerEnabled = true;
  public static final boolean deployEnabled = true;
  public static final boolean flywheelEnabled = true;
  public static final boolean indexerEnabled = true;
  public static final boolean spindexerEnabled = true;
  public static final boolean ledEnabled = false;

  // Controllers
  private final DriverMapper driver = new DriverMapperComp();
  private final OperatorMapper operator = new OperatorMapperTest();

  // Subsystems
  private final DriveWrapper drive = new DriveWrapper(SubsystemFactory.buildDrive());

  @SuppressWarnings("unused")
  private Vision vision;

  private ObjectDetection objectDetection;
  private Turret turret;
  private Hood hood;
  private Intake intake;
  private Flywheel flywheel;
  private Led led;

  // Subsystem Enabled // only matters for real robot
  private static final boolean driveEnabled = true;
  private static final boolean visionEnabled = true;
  private static final boolean objectDetectionEnabled = true;
  private static final boolean turretEnabled = true;
  private static final boolean hoodEnabled = true;
  private static final boolean intakeEnabled = true;
  private static final boolean flywheelEnabled = true;
  private static final boolean ledEnabled = true;

  /* Sim Booleans */
  private static final boolean isThreaded = false;
  // Set whether models should exist in physics engine
  private static final boolean simulateShooterBase = true;
  private static final boolean simulateTurret = true;
  private static final boolean simulateHood = true;

  private SwerveDriveSimulation3D driveSimulation;

  // Alerts
  private final Alert primaryDisconnected =
      new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected =
      new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);

  // Dashboard inputs
  private SendableChooser<Command> autoChooser;

  private LoggedTunableNumber flywheelSpeed = new LoggedTunableNumber("FlywheelVelocity", 7500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        if (driveEnabled) {
          drive =
              new Drive(
                  new DriveIOReal(),
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                  new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                  new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                  new ModuleIOTalonFXReal(TunerConstants.BackRight),
                  (pose) -> {});
        }
        if (visionEnabled) {
          vision =
              new Vision(
                  drive,
                  new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                  new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        }
        if (turretEnabled) {
          turret = new Turret(new TurretIOSparkFlex());
        }
        if (hoodEnabled) {
          hood = new Hood(new HoodIOSparkFlex());
        }
        if (intakeEnabled) {
          intake = new Intake(new IntakeIOTalonFXS());
        }
        if (flywheelEnabled) {
          flywheel = new Flywheel(new FlywheelIOSparkFlex());
        }
        if (objectDetectionEnabled) {
          // TODO: add actual limelight name
          objectDetection =
              new ObjectDetection(
                  new ODIOReal(
                      "limelight",
                      ObjectDetection.GamePieceType.FUEL,
                      LimelightConfig.LIMELIGHT_3G));
        }
        if (ledEnabled && flywheelEnabled && driveEnabled && intakeEnabled) {
          led = new Led(new LedIOBlinkin());
        }
      }
      case TEST -> {
        // Real robot, instantiate hardware IO implementations
        Drive.driveTestMultiplier = 0.2;
        if (driveEnabled) {
          drive =
              new Drive(
                  new DriveIOReal(),
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                  new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                  new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                  new ModuleIOTalonFXReal(TunerConstants.BackRight),
                  (pose) -> {});
        }
        if (visionEnabled) {
          vision =
              new Vision(
                  drive,
                  new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                  new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        }
        if (turretEnabled) {
          turret = new Turret(new TurretIOSparkFlex());
        }
        if (hoodEnabled) {
          hood = new Hood(new HoodIOSparkFlex());
        }
        if (intakeEnabled) {
          intake = new Intake(new IntakeIOSparkFlex());
        }
        if (flywheelEnabled) {
          flywheel = new Flywheel(new FlywheelIOSparkFlex());
        }
        if (objectDetectionEnabled) {
          // TODO: add actual limelight name
          objectDetection =
              new ObjectDetection(
                  new ODIOReal(
                      "limelight",
                      ObjectDetection.GamePieceType.FUEL,
                      LimelightConfig.LIMELIGHT_3G));
        }
        if (ledEnabled && flywheelEnabled && driveEnabled && intakeEnabled) {
          led = new Led(new LedIOBlinkin());
        }
      }
      case SIM -> {
        // TODO: Add option to choose between 2d and 3d sim
        // Sim robot, instantiate physics sim IO implementations

        // Initialize Rebuilt arena
        if (isThreaded) {
          new Arena2026Rebuilt3D(PhysicsThreadConfig.enabled(100));
        } else {
          new Arena2026Rebuilt3D();
        }

        // Initialize robot simulation
        // TODO: Reduce drive simulation drifting
        RobotSimulation3D robotSimulation =
            new RobotSimulation3D(
                new SwerveDriveSimulation3D(Drive.getMapleSimConfig(), Pose2d.kZero),
                new Pose2d(14, 3, Rotation2d.kZero));
        driveSimulation = (SwerveDriveSimulation3D) robotSimulation.getDriveSimulation();
        robotSimulation.addIntakeSimulation(
            "Fuel", 0.1, Constants.RobotConstants.kBumperWidthMeters, 0.1, 35);
        intakeSimulation = robotSimulation.getIntakeSimulation();

        if (simulateShooterBase) {
          robotSimulation.addShooterBase(new MeshSimulation3D("model_0.obj", 0.1));
          if (simulateTurret) {
            robotSimulation.addTurret(new MeshSimulation3D("model_1.obj", 0.1));
            if (simulateHood) {
              robotSimulation.addHood(new MeshSimulation3D("model_2.obj", 0.1));
            }
          }
        }

        drive =
            new Drive(
                new DriveIOSim3D(driveSimulation),
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                // TODO: ModuleIOSim does not work
                // new ModuleIOSim(driveSimulation.getModules()[0]),
                // new ModuleIOSim(driveSimulation.getModules()[1]),
                // new ModuleIOSim(driveSimulation.getModules()[2]),
                // new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        hood = new Hood(new HoodIOTest(robotSimulation));

        turret = new Turret(new TurretIOSim(hood::getAngle, drive, robotSimulation));

        objectDetection =
            new ObjectDetection(
                new ODIOSimInView(
                    "Fuel",
                    LimelightConfig.LIMELIGHT_3G,
                    4,
                    driveSimulation::getSimulatedDriveTrainPose3dGroundRelative,
                    () -> new Pose3d(Translation3d.kZero, new Rotation3d())));

        intake = new Intake(new IntakeIOSim3D(intakeSimulation));

        flywheel =
            new Flywheel(
                new FlywheelIOSim3D(
                    driveSimulation,
                    intakeSimulation,
                    hood::getAngle,
                    () -> Rotation2d.fromRadians(turret.getTurretAngle()),
                    robotSimulation));
      }

      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new DriveIO() {},
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
      }
    }

    // Initialize drive suppliers
    switch (Constants.controlMode) {
      case JOYSTICK -> {
        leftX = () -> -driverController.getLeftX();
        leftY = () -> -driverController.getLeftY();
        rightX = () -> -driverController.getRightX();
        rightY = () -> -driverController.getRightY();
      }
      case KEYBOARD -> {
        leftX = keyboardHID::leftX;
        leftY = keyboardHID::leftY;
        rightX = keyboardHID::rightX;
        rightY = keyboardHID::rightY;
      }
    }

    // Initialize commands
    // TODO: runIntakeCommand should deploy intake before it can run it
    if (intakeEnabled) {
      runIntakeCommand =
          Commands.runEnd(() -> intake.setRelativeVelocity(1), () -> intake.setRelativeVelocity(0));
      NamedCommands.registerCommand("runIntake", runIntakeCommand);
    } else {
      NamedCommands.registerCommand("runIntake", Commands.none());
    }
    if (driveEnabled && flywheelEnabled && turretEnabled && hoodEnabled) {
      shootCommand = TurretCommands.shootCommand(flywheel, turret, hood, drive);
      shootToHubCommand = TurretCommands.shootToHubCommand(flywheel, turret, hood, drive);
      shuttleCommand = TurretCommands.shuttleCommand(flywheel, turret, hood, drive);
      NamedCommands.registerCommand("shoot", shootToHubCommand);
      NamedCommands.registerCommand("shuttle", shuttleCommand);
    } else {
      NamedCommands.registerCommand("shoot", Commands.none());
      NamedCommands.registerCommand("shuttle", Commands.none());
    }
    if (driveEnabled && objectDetectionEnabled && intakeEnabled) {
      driveAndIntakeCommand =
          AutonCommands.driveToAndIntakeGamePieces(
              drive, objectDetection, intake, leftX, leftY, rightX, rightY);
      NamedCommands.registerCommand("driveAndIntake", driveAndIntakeCommand);
    } else {
      NamedCommands.registerCommand("driveAndIntake", Commands.none());
    }
    if (driveEnabled && objectDetectionEnabled && turretEnabled && flywheelEnabled && hoodEnabled) {
      driveAndScoreCommand =
          AutonCommands.driveToAndShootIntoHub(
              drive, turret, flywheel, hood, objectDetection, leftX, leftY, rightX, rightY);
      NamedCommands.registerCommand("driveAndScore", driveAndScoreCommand);
    } else {
      NamedCommands.registerCommand("driveAndScore", Commands.none());
    }
    if (driveEnabled) {
      setTurnMode = DriveCommands.setTurnMode(drive);
    }

    // Set up auto routines
    autoChooser = new SendableChooser<Command>();

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization());
    autoChooser.addOption("Drive Simple FF Characterization", drive.feedforwardCharacterization());
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Angular)",
        drive.sysIdQuasistaticAngular(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynam ic Angular)",
        drive.sysIdDynamicAngular(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Left Auto", AutoBuilder.buildAuto("LeftAuto"));
    autoChooser.addOption("Left Auto with Depot", AutoBuilder.buildAuto("LeftAutoWithDepot"));
    autoChooser.addOption("Mid Auto", AutoBuilder.buildAuto("MidAuto"));
    autoChooser.addOption("Right Auto", AutoBuilder.buildAuto("RightAuto"));
    autoChooser.addOption("Right Auto Two Cycle", AutoBuilder.buildAuto("RightAutoTwoCycle"));

    autoChooser.addOption("Left Just Shoot", AutoBuilder.buildAuto("LeftJustShoot"));
    autoChooser.addOption("Mid Just Shoot", AutoBuilder.buildAuto("MidJustShoot"));
    autoChooser.addOption("Right Just Shoot", AutoBuilder.buildAuto("RightJustShoot"));

    autoChooser.addOption("Do Nothing", AutoBuilder.buildAuto("DoNothing"));
    autoChooser.addOption("Just Shoot", AutoBuilder.buildAuto("JustShoot"));

    autoChooser.addOption("Test Auto", AutoBuilder.buildAuto("TestAuto"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver */

    // Drive
    drive.setDefaultCommand(
        drive.driveCommand(
            driver::getTranslationX,
            driver::getTranslationY,
            driver::getRotationX,
            driver::getRotationY));

    driver.getTurnModeToggleButton().onTrue(drive.setTurnModeCommand());
    driver.getStopWithXButton().onTrue(drive.stopWithXCommand());
    driver.getZeroPoseButton().onTrue(drive.zeroPoseCommand());

    driver.getRollerToggleButton().onTrue(roller.toggleCommand());

    driver.getDeployButton().whileTrue(superstructure.deployCommand());
    driver.getRetractButton().whileTrue(superstructure.retractCommand());

    driver.getShootButton().whileTrue(superstructure.shootCommand());

    /* Operator */

        turret.setDefaultCommand(
            Commands.run(() -> turret.setTurretVelocity(keyboardHID.middleX()), turret));

        keyboardHID
            .z()
            .whileTrue(LimelightHelpersCommands.alignHeadingCommand(drive, objectDetection));

        keyboardHID
            .x()
            .whileTrue(
                Commands.runEnd(
                    () -> flywheel.setVelocity(FlywheelConstants.maxRPM),
                    () -> flywheel.setVelocity(0)));

        keyboardHID.c().whileTrue(shootToHubCommand);

        keyboardHID.v().whileTrue(runIntakeCommand);

        keyboardHID.m().onTrue(driveAndIntakeCommand);

        keyboardHID.comma().whileTrue(driveAndScoreCommand);

        keyboardHID.period().whileTrue(shuttleCommand);

        // Outpost dump
        keyboardHID
            .slash()
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Arena2026Rebuilt3D rebuiltArena =
                          (Arena2026Rebuilt3D) SimulatedArena3D.getInstance();
                      Optional<Alliance> allianceOptional = DriverStation.getAlliance();
                      if (allianceOptional.isPresent()) {
                        rebuiltArena.outpostDump(allianceOptional.get() == Alliance.Blue);
                      }
                    }));
      }
      default -> {}
    }

    if (driveEnabled) {
      // Switch to X pattern when X button is pressed
      driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

      // Reset gyro / odometry
      final Runnable resetGyro =
          Constants.currentMode == Constants.Mode.SIM
              ? () ->
                  drive.setPose(
                      driveSimulation
                          .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
              : () ->
                  drive.setPose(
                      new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
      driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    }
    SmartDashboard.putData("SeqSystemCheck", AdvancedSubsystem.sequentialCheck());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update from HubShiftUtil
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
    SmartDashboard.putString(
        "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());

    // Controller disconnected alerts
    primaryDisconnected.set(!DriverStation.isJoystickConnected(MapperConstants.driverPort));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(MapperConstants.operatorPort));
  }

  @SuppressWarnings("unused")
  private boolean driverHasInput() {
    return Math.abs(driver.getTranslationX()) >= DriveWrapper.deadband
        || Math.abs(driver.getTranslationY()) >= DriveWrapper.deadband
        || Math.abs(driver.getRotationX()) >= DriveWrapper.deadband
        || Math.abs(driver.getRotationY()) >= DriveWrapper.deadband;
  }
}
