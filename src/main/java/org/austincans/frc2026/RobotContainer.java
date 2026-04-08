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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import org.austincans.frc2026.FieldConstants.AprilTagLayoutType;
import org.austincans.frc2026.commands.AutonCommands;
import org.austincans.frc2026.commands.DriveCommands;
import org.austincans.frc2026.commands.LimelightHelpersCommands;
import org.austincans.frc2026.commands.TurretCommands;
import org.austincans.frc2026.commands.shooter.ShooterCommands;
import org.austincans.frc2026.generated.TunerConstants;
import org.austincans.frc2026.subsystems.drive.*;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.flywheel.FlywheelConstants;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIOSparkFlex;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.hood.HoodIOServoLinear;
import org.austincans.frc2026.subsystems.intake.Intake;
import org.austincans.frc2026.subsystems.intake.IntakeIOSparkFlex;
import org.austincans.frc2026.subsystems.intake.deploy.DeployIOSparkFlex;
import org.austincans.frc2026.subsystems.led.Led;
import org.austincans.frc2026.subsystems.led.LedIOBlinkin;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretIOSparkFlex;
import org.austincans.frc2026.subsystems.vision.*;
import org.austincans.frc2026.subsystems.vision.VisionConstants.LimelightConfig;
import org.austincans.frc2026.util.HubShiftUtil;
import org.austincans.frc2026.util.LoggedTunableNumber;
import org.austincans.lib.util.KeyboardHID;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystem Enabled // only matters for real robot
  private static final boolean driveEnabled = true;
  private static final boolean visionEnabled = true;
  private static final boolean objectDetectionEnabled = false;
  private static final boolean turretEnabled = true;
  private static final boolean hoodEnabled = true;
  private static final boolean intakeEnabled = true;
  private static final boolean flywheelEnabled = true;
  private static final boolean ledEnabled = false;

  // Subsystems
  private Drive drive;
  @SuppressWarnings("unused")
  private Vision vision;
  private ObjectDetection objectDetection;
  private Turret turret;
  private Hood hood;
  private Intake intake;
  private Flywheel flywheel;
  @SuppressWarnings("unused")
  private Led led;

  // Alerts
  private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected = new Alert("Secondary controller disconnected (port 1).",
      AlertType.kWarning);

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final KeyboardHID keyboardHID = new KeyboardHID();

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  private DoubleSupplier leftY, leftX, rightY, rightX;

  // Commands
  private Command toggleIntakeCommand;
  private Command driveAndIntakeCommand;
  private Command driveAndScoreCommand;
  private Command shootCommand;
  private Command setTurnMode;
  // auton only
  private Command shuttleCommand;
  private Command shootToHubCommand;
  private Command otherShootCommand;

  LoggedTunableNumber flywheelSpeed = new LoggedTunableNumber("FlywheelVelocity", 7500.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  @SuppressWarnings("unused")
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Logger.end();
        // Real robot, instantiate hardware IO implementations
        // new VisionIOLimelightTurret("limelight-turret", drive::getRotation, drive));
        if (driveEnabled) {
          if (turretEnabled) {
            turret = new Turret(new TurretIOSparkFlex());
            drive = new Drive(
                new DriveIOReal(),
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {
                });
          } else {
            drive = new Drive(
                new DriveIOReal(),
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {
                });
          }
        }
        if (visionEnabled) {
          vision = new Vision(
              drive,
              new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
              new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
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
          objectDetection = new ObjectDetection(
              new ODIOReal(
                  "limelight",
                  ObjectDetection.GamePieceType.FUEL,
                  LimelightConfig.LIMELIGHT_3G));
        }
        if (ledEnabled && flywheelEnabled && driveEnabled && intakeEnabled) {
          led = new Led(new LedIOBlinkin());
        }
      }
      default -> {
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new DriveIO() {
            },
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            (pose) -> {
            });
        vision = new Vision(drive, new VisionIO() {
        }, new VisionIO() {
        });
      }
    }

    // Initialize drive suppliers
    switch (Constants.controlMode) {
      case JOYSTICK -> {
        leftX = () -> -driverController.getLeftX() * Drive.driveTestMultiplier;
        leftY = () -> -driverController.getLeftY() * Drive.driveTestMultiplier;
        rightX = () -> -driverController.getRightX() * Drive.driveTestMultiplier * Drive.turningMultiplier;
        rightY = () -> -driverController.getRightY() * Drive.driveTestMultiplier;
      }
      case KEYBOARD -> {
        leftX = keyboardHID::leftX;
        leftY = keyboardHID::leftY;
        rightX = keyboardHID::rightX;
        rightY = keyboardHID::rightY;
      }
    }

    /* Initialize commands */

    // TODO: runIntakeCommand should deploy intake before it can run it
    if (intakeEnabled) {
      toggleIntakeCommand = intake.toggleIntakeCommand();

      Command runIntakeCommand = Commands.runEnd(
          () -> intake.setRelativeVelocity(0.3), () -> intake.setRelativeVelocity(0));
      NamedCommands.registerCommand("runIntake", runIntakeCommand);
    } else {
      NamedCommands.registerCommand("runIntake", Commands.none());
    }
    if (driveEnabled && flywheelEnabled && turretEnabled && hoodEnabled) {
      shootCommand = ShooterCommands.shootCommand(flywheel, turret, hood, drive)
          .finallyDo(() -> turret.setDesiredAngleTurretRelative(Rotation2d.kZero));
      otherShootCommand = Commands.run(
          () -> {
            flywheel.runIndexerAndFlywheel(flywheelSpeed.getAsDouble());
          },
          flywheel)
          .finallyDo(
              () -> {
                flywheel.stopMotors();
              });
      shuttleCommand = ShooterCommands.shuttleCommand(drive, flywheel, hood, turret);
      NamedCommands.registerCommand(
          "shoot", ShooterCommands.shootToHubCommand(drive, flywheel, hood, turret));
      NamedCommands.registerCommand("shuttle", shuttleCommand);
    } else {
      NamedCommands.registerCommand("shoot", Commands.none());
      NamedCommands.registerCommand("shuttle", Commands.none());
    }
    if (driveEnabled && objectDetectionEnabled && intakeEnabled) {
      driveAndIntakeCommand = AutonCommands.driveToAndIntakeGamePieces(
          drive, objectDetection, intake, leftX, leftY, rightX, rightY);
      NamedCommands.registerCommand("driveAndIntake", driveAndIntakeCommand);
    } else {
      NamedCommands.registerCommand("driveAndIntake", Commands.none());
    }
    if (driveEnabled && objectDetectionEnabled && turretEnabled && flywheelEnabled && hoodEnabled) {
      driveAndScoreCommand = AutonCommands.driveToAndShootIntoHub(
          drive, turret, flywheel, hood, objectDetection, leftX, leftY, rightX, rightY);
      NamedCommands.registerCommand("driveAndScore", driveAndScoreCommand);
    } else {
      NamedCommands.registerCommand("driveAndScore", Commands.none());
    }
    if (driveEnabled) {
      setTurnMode = DriveCommands.setTurnMode();
    }

    // Set up auto routines
    autoChooser = new SendableChooser<Command>();

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
        "Drive SysId (Dynam ic Angular)",
        drive.sysIdDynamicAngular(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Top Auto", AutoBuilder.buildAuto("Top Auto"));
    autoChooser.addOption("Mid Auto", AutoBuilder.buildAuto("Mid Auto"));
    autoChooser.addOption("Shuttling Auto", AutoBuilder.buildAuto("Shuttling Auto"));

    autoChooser.addOption("Blue Just Shoot Top", AutoBuilder.buildAuto("blueTopJustShoot"));
    autoChooser.addOption("Blue Just Shoot Mid", AutoBuilder.buildAuto("blueMidJustShoot"));
    autoChooser.addOption("Blue Just Shoot Bot", AutoBuilder.buildAuto("blueBottomJustShoot"));

    autoChooser.addOption("Auto1", AutoBuilder.buildAuto("Auto1"));
    autoChooser.addOption("Auto2", AutoBuilder.buildAuto("Auto2"));
    autoChooser.addOption("Auto3", AutoBuilder.buildAuto("Auto3"));

    autoChooser.addOption("Do nothing", AutoBuilder.buildAuto("Do Nothing"));
    autoChooser.addOption("Just Shoot", AutoBuilder.buildAuto("justShoot"));
    // Default command, normal field-relative drive
    if (driveEnabled) {
      drive.setDefaultCommand(DriveCommands.driveCommand(drive, leftY, leftX, rightY, rightX));
    }

    // Configure the button bindings
    configureButtonBindings();

    autoChooser.addOption("shoot", AutoBuilder.buildAuto("shoot"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Command pointToHubCommand;

    switch (Constants.controlMode) {
      case JOYSTICK -> {
        if (driveEnabled) {
          driverController.leftBumper().onTrue(toggleIntakeCommand);
          operatorController
              .leftBumper()
              .onTrue(Commands.runOnce(() -> intake.setRelativeVelocity(-0.35), intake));
          operatorController
              .rightBumper()
              .onTrue(
                  Commands.runOnce(
                      () -> {
                        ShooterCommands.staticShooter = !ShooterCommands.staticShooter;
                      }));

          // Switch to X pattern when X button is pressed
          driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

          if (flywheelEnabled) {
            // TODO: add flywheel tracking command (indexer at zero speed, flywheel at
            // desired
            // speed)
            operatorController.povDown().whileTrue(otherShootCommand);
            driverController.rightBumper().whileTrue(shootCommand);
          }
          if (setTurnMode != null) {
            driverController.b().onTrue(setTurnMode);
          }

          if (intakeEnabled) {
            driverController
                .leftTrigger()
                .onTrue(intake.deployCommand())
                // .onTrue(intake.toggleDeployCommand())
                .onFalse(intake.stopDeployCommand());

            driverController
                .rightTrigger()
                .onTrue(intake.retractCommand())
                .onFalse(intake.endRetractCommand());
          }
          if (hoodEnabled) {
            // TESTING
            // operatorController.x().onTrue(hood.addToPulseCommand(50));
            operatorController.x().onTrue(Commands.runOnce(() -> hood.setPulse(1000)));
            // operatorController.b().onTrue(hood.addToPulseCommand(-50));

            // operatorController.leftBumper().onTrue(hood.setPulseCommand(1000));
            // operatorController.rightBumper().onTrue(hood.setPulseCommand(1500));

            NamedCommands.registerCommand(
                "turretAngleFar", Commands.runOnce(() -> hood.setPulse(1150), hood));
          } else {
            NamedCommands.registerCommand("turretAngleFar", Commands.none());
          }
          if (turretEnabled) {
            pointToHubCommand = TurretCommands.pointToHub(turret, drive);

            operatorController.povDown().whileTrue(pointToHubCommand);
            operatorController
                .povUp()
                .whileTrue(
                    Commands.run(() -> flywheel.runIndexerAndFlywheel(flywheelSpeed.getAsDouble())))
                .onFalse(Commands.runOnce(() -> flywheel.stopMotors()));

            turret.setDefaultCommand(
                TurretCommands.joystickTurretAngleCommand(
                    turret,
                    drive,
                    () -> -operatorController.getRightY(),
                    () -> -operatorController.getRightX()));
            operatorController.y().onTrue(turret.resetAngle());
            NamedCommands.registerCommand("point to hub", pointToHubCommand);
          } else {
            NamedCommands.registerCommand("point to hub", Commands.none());
          }

          // turret and hood
          // UNTESTED
          if (turretEnabled && hoodEnabled) {
            // operatorController.a().whileTrue(ShooterCommands.trackCommand(turret, hood,
            // drive));
          }

          // Reset gyro / odometry

          final Runnable resetGyro = () -> drive.setPose(
              new Pose2d(
                  drive.getPose().getTranslation(),
                  DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                      ? Rotation2d.k180deg
                      : Rotation2d.kZero)); // zero gyro
          driverController.back().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        }
      }
      // case KEYBOARD -> {

      // turret.setDefaultCommand(
      // Commands.run(() -> turret.setTurretVelocity(keyboardHID.middleX()), turret));

      // keyboardHID
      // .z()
      // .whileTrue(LimelightHelpersCommands.alignHeadingCommand(drive,
      // objectDetection));

      // keyboardHID
      // .x()
      // .whileTrue(
      // Commands.runEnd(
      // () -> flywheel.setVelocity(FlywheelConstants.maxRPM),
      // () -> flywheel.setVelocity(0)));

      // keyboardHID.c().whileTrue(shootToHubCommand);

      // keyboardHID.v().onTrue(toggleIntakeCommand);

      // keyboardHID.m().onTrue(driveAndIntakeCommand);

      // keyboardHID.comma().whileTrue(driveAndScoreCommand);

      // keyboardHID.period().whileTrue(shuttleCommand);
      // }
      default -> {
      }
    }

    SmartDashboard.putData("SeqSystemCheck", AdvancedSubsystem.sequentialCheck());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return AutoBuilder.buildAuto("shoot");
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
    primaryDisconnected.set(!DriverStation.isJoystickConnected(driverController.getHID().getPort()));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(operatorController.getHID().getPort()));
  }
}
