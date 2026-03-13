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

import static org.austincans.frc2026.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.austincans.frc2026.commands.AutonCommands;
import org.austincans.frc2026.commands.DriveCommands;
import org.austincans.frc2026.commands.LimelightHelpersCommands;
import org.austincans.frc2026.commands.TurretCommands;
import org.austincans.frc2026.generated.TunerConstants;
import org.austincans.frc2026.simulation.RobotSimulation3D;
import org.austincans.frc2026.subsystems.battery.BatterySubsystem;
import org.austincans.frc2026.subsystems.drive.*;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.flywheel.FlywheelConstants;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIOSim3D;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIOSparkFlex;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.hood.HoodIOServoLinear;
import org.austincans.frc2026.subsystems.hood.HoodIOSparkFlex;
import org.austincans.frc2026.subsystems.hood.HoodIOTest;
import org.austincans.frc2026.subsystems.intake.Intake;
import org.austincans.frc2026.subsystems.intake.IntakeIOSim3D;
import org.austincans.frc2026.subsystems.intake.IntakeIOSparkFlex;
import org.austincans.frc2026.subsystems.intake.deploy.DeployIOFake;
import org.austincans.frc2026.subsystems.intake.deploy.DeployIOSparkFlex;
import org.austincans.frc2026.subsystems.led.Led;
import org.austincans.frc2026.subsystems.led.LedIOBlinkin;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretIOSim;
import org.austincans.frc2026.subsystems.turret.TurretIOSparkFlex;
import org.austincans.frc2026.subsystems.vision.*;
import org.austincans.frc2026.subsystems.vision.VisionConstants.LimelightConfig;
import org.austincans.lib.util.KeyboardHID;
import org.ironmaple.simulation.IntakeSimulation3D;
import org.ironmaple.simulation.MeshSimulation3D;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation3D;
import org.ironmaple.simulation.physics.threading.PhysicsThreadConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt3D;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;

  @SuppressWarnings("unused")
  private Vision vision;

  private ObjectDetection objectDetection;
  private BatterySubsystem batterySubsystem;
  private Turret turret;
  private Hood hood;
  private Intake intake;
  private Flywheel flywheel;
  private Led led;

  // Subsystem Enabled // only matters for real robot
  private static final boolean driveEnabled = true;
  private static final boolean visionEnabled = true;
  private static final boolean objectDetectionEnabled = false;
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

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final KeyboardHID keyboardHID = new KeyboardHID();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private IntakeSimulation3D intakeSimulation;

  private DoubleSupplier leftY, leftX, rightY, rightX;

  // Commands
  private Command runIntakeCommand;
  private Command driveAndIntakeCommand;
  private Command driveAndScoreCommand;
  private Command shootCommand;
  private Command setTurnMode;
  // auton only
  private Command shuttleCommand;
  private Command shootToHubCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("unused")
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        batterySubsystem = new BatterySubsystem();
        if (driveEnabled) {
          if (turretEnabled) {
            turret = new Turret(new TurretIOSparkFlex());
            drive =
                new Drive(
                    new DriveIOReal(),
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                    new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                    new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                    new ModuleIOTalonFXReal(TunerConstants.BackRight),
                    (pose) -> {},
                    () -> {
                      return Rotation2d.fromRadians(turret.getTurretAngle());
                    });
          } else {
            drive =
                new Drive(
                    new DriveIOReal(),
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                    new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                    new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                    new ModuleIOTalonFXReal(TunerConstants.BackRight),
                    (pose) -> {},
                    () -> {
                      return Rotation2d.kZero;
                    });
          }
        }
        if (visionEnabled) {
          // toward back of robot is positive x
          // right is positive
          // up is positive z
          Translation3d robotCenterToTurretCenterWithCameraHeightMeters =
              new Translation3d(
                  Units.inchesToMeters(8.140 - (6.875 / 2)), 0, Units.inchesToMeters(18.375 + 5.5));
          double turretHorizontalDistanceToCameraMeters = Units.inchesToMeters(4.84 + (6.875 / 2));
          vision =
              new Vision(
                  drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation)
                  // new VisionIOPhotonVision(
                  // VisionConstants.camera1Name,
                  // () -> {
                  // Transform3d robotToTurret =
                  // new Transform3d(
                  // robotCenterToTurretCenterWithCameraHeightMeters,
                  // Rotation3d.kZero);
                  // Transform3d turretCenterToCamera =
                  // new Transform3d(
                  // new Translation3d(turretHorizontalDistanceToCameraMeters, 0, 0),
                  // new Rotation3d(0, 0, turret.getTurretAngle()));
                  // return robotToTurret.plus(turretCenterToCamera);
                  );
        }
        if (hoodEnabled) {
          hood = new Hood(new HoodIOServoLinear());
        }
        if (intakeEnabled) {
          intake = new Intake(new IntakeIOSparkFlex(), new DeployIOSparkFlex());
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
                  (pose) -> {},
                  () -> Rotation2d.kZero);
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
          intake = new Intake(new IntakeIOSparkFlex(), new DeployIOSparkFlex());
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
                driveSimulation::setSimulationWorldPose,
                () -> Rotation2d.kZero);

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, () -> robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name,
                    () -> robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose));

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

        intake = new Intake(new IntakeIOSim3D(intakeSimulation), new DeployIOFake());

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
                (pose) -> {},
                () -> Rotation2d.kZero);
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
      }
    }

    // Initialize drive suppliers
    switch (Constants.controlMode) {
      case JOYSTICK -> {
        leftX = () -> -driverController.getLeftX() / Drive.driveTestMultiplier;
        leftY = () -> -driverController.getLeftY() / Drive.driveTestMultiplier;
        rightX = () -> -driverController.getRightX() / Drive.driveTestMultiplier;
        rightY = () -> -driverController.getRightY() / Drive.driveTestMultiplier;
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
          Commands.runEnd(
              () -> intake.setRelativeVelocity(0.5), () -> intake.setRelativeVelocity(0));
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
    autoChooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("Top Auto"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
        "Drive SysId (Dynamic Angular)",
        drive.sysIdDynamicAngular(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Top Auto", AutoBuilder.buildAuto("Top Auto"));
    autoChooser.addOption("Mid Auto", AutoBuilder.buildAuto("Mid Auto"));
    autoChooser.addOption("Shuttling Auto", AutoBuilder.buildAuto("Shuttling Auto"));

    // Default command, normal field-relative drive
    if (driveEnabled) {
      drive.setDefaultCommand(
          DriveCommands.driveCommand(
              drive,
              () -> -driverController.getLeftY(),
              () -> -driverController.getLeftX(),
              () -> driverController.getRightX(),
              () -> driverController.getRightY()));
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Lock to 0° when A button is held
    // controller
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> new Rotation2d()));

    switch (Constants.controlMode) {
      case JOYSTICK -> {
        if (driveEnabled) {
          driverController.leftBumper().whileTrue(runIntakeCommand);
          // Switch to X pattern when X button is pressed
          driverController.back().onTrue(Commands.runOnce(() -> drive.resetGyro()));
          driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
          if (flywheelEnabled) {
            driverController
                .rightBumper()
                .whileTrue(
                    Commands.runEnd(
                        () -> {
                          flywheel.runIndexerAndFlywheel(7500);
                          System.out.println("RAN FLYWHEEL");
                        },
                        () -> {
                          flywheel.stopMotors();
                        },
                        flywheel));
          }
          if (setTurnMode != null) {
            driverController.b().onTrue(setTurnMode);
          }

          if (intakeEnabled) {
            operatorController
                .leftBumper()
                .whileTrue(
                    Commands.runEnd(
                        () -> {
                          System.out.println("deploy positive direction");
                          intake.deploy();
                        },
                        intake::stopDeploy, // intake.runDeployVelocity(0)
                        intake));
            operatorController
                .rightBumper()
                .whileTrue(
                    Commands.runEnd(
                        () -> {
                          System.out.println("deploy negative direction");
                          intake.retract();
                        },
                        intake::stopDeploy, // intake.runDeployVelocity(0)
                        intake));

            // operatorController.x().onTrue(intake.deployCommand());
            // operatorController.b().onTrue(intake.retractCommand());
          }
          if (hoodEnabled) {
            operatorController.x().onTrue(Commands.runOnce(() -> hood.setHood(1000), hood));
            operatorController.b().onTrue(Commands.runOnce(() -> hood.setHood(1750), hood));
          }
          if (turretEnabled) {
            operatorController.leftTrigger().onTrue(TurretCommands.pointToHub(turret, drive));

            turret.setDefaultCommand(
                TurretCommands.joystickTurretAngleCommand(
                    turret, drive, operatorController::getRightY, operatorController::getRightX));
            // operatorController
            //     .rightTrigger()
            //     .onTrue(turret.setTurretAngleCommand(Rotation2d.fromDegrees(90)));
            // operatorController
            //     .start()
            //     .onTrue(turret.setTurretAngleCommand(Rotation2d.fromDegrees(-90)));
          }

          // Reset gyro / odometry

          // final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
          //     ? () -> drive.setPose(
          //         driveSimulation
          //             .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose
          //     // during simulation
          //     : () -> drive.setPose(
          //         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
          // driverController.start().onTrue(Commands.runOnce(resetGyro,
          // drive).ignoringDisable(true));
        }
      }
      case KEYBOARD -> {
        hood.setDefaultCommand(Commands.run(() -> hood.setVelocity(keyboardHID.middleY()), hood));

        turret.setDefaultCommand(
            Commands.run(() -> turret.setTurretVelocity(keyboardHID.middleX()), turret));

        keyboardHID
            .z()
            .whileTrue(LimelightHelpersCommands.alignHeadingCommand(drive, objectDetection));

        keyboardHID
            .x()
            .whileTrue(
                Commands.runEnd(
                    () -> flywheel.setRelativeVelocity(FlywheelConstants.maxRPM),
                    () -> flywheel.setRelativeVelocity(0)));

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
    SmartDashboard.putData("SeqSystemCheck", AdvancedSubsystem.sequentialCheck());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(14, 3, new Rotation2d()));
    SimulatedArena3D.getInstance().resetFieldForAuto();
  }

  public void preloadSimulation() {
    if (intakeSimulation != null) {
      intakeSimulation.addGamePiecesToIntake(8);
    }
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena3D.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition",
        driveSimulation.getSimulatedDriveTrainPose3dGroundRelative());
    Logger.recordOutput(
        "FieldSimulation/Fuel",
        SimulatedArena3D.getInstance().getGamePiecePosesArrayByType("Fuel"));
    // Logger.recordOutput(
    // "FieldSimulation/Fuel2D",
    // SimulatedArena.getInstance().getGamePiecePosesArrayByType("Fuel"));
  }
}
