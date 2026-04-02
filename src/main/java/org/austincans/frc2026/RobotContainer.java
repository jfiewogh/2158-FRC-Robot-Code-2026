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
import org.austincans.frc2026.util.LoggedTunableNumber;
import org.austincans.lib.util.KeyboardHID;

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
  private static final boolean ledEnabled = false;

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

  LoggedTunableNumber flywheelSpeed = new LoggedTunableNumber("FlywheelVelocity", 7500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("unused")
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Logger.end();
        // Real robot, instantiate hardware IO implementations
        if (driveEnabled) {
          if (turretEnabled) {
            turret = new Turret(new TurretIOSparkFlex());
            // new VisionIOLimelightTurret("limelight-turret", drive::getRotation, drive));
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
                      return Rotation2d.fromRadians(turret.getTurretRads());
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
          // Translation3d robotCenterToTurretCenterWithCameraHeightMeters =
          // new Translation3d(
          // Units.inchesToMeters(8.140 - (6.875 / 2)), 0, Units.inchesToMeters(18.375 +
          // 5.5));
          // double turretHorizontalDistanceToCameraMeters = Units.inchesToMeters(4.84 +
          // (6.875 /
          // 2));
          vision =
              new Vision(
                  drive,
                  new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                  new VisionIOLimelight(VisionConstants.camera1Name, turret::getTurretRotation2d)
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
        leftX = () -> -driverController.getLeftX() * Drive.driveTestMultiplier;
        leftY = () -> -driverController.getLeftY() * Drive.driveTestMultiplier;
        rightX =
            () ->
                -driverController.getRightX() * Drive.driveTestMultiplier * Drive.turningMultiplier;
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

      Command runIntakeCommand =
          Commands.runEnd(
              () -> intake.setRelativeVelocity(0.3), () -> intake.setRelativeVelocity(0));
      NamedCommands.registerCommand("runIntake", runIntakeCommand);
    } else {
      NamedCommands.registerCommand("runIntake", Commands.none());
    }
    if (driveEnabled && flywheelEnabled && turretEnabled && hoodEnabled) {
      shootCommand =
          ShooterCommands.shootCommand(flywheel, turret, hood, drive)
              // ShooterCommands.shootToHubCommand(drive, flywheel, hood, turret)
              .finallyDo(() -> turret.setDesiredAngleRobotRelative(Rotation2d.kZero));
      // shootCommand =
      //     Commands.run(
      //             () -> {
      //               flywheel.runIndexerAndFlywheel(flywheelSpeed.getAsDouble());
      //             },
      //             flywheel)
      //         .finallyDo(
      //             () -> {
      //               flywheel.stopMotors();
      //             })
      //         .withTimeout(5);

      // this is temporary
      Command otherShootCommand =
          Commands.run(
                  () -> {
                    flywheel.runIndexerAndFlywheel(7500);
                  },
                  flywheel)
              .finallyDo(
                  () -> {
                    flywheel.stopMotors();
                  })
              .withTimeout(5);

      // shootToHubCommand = TurretCommands.shootToHubCommand(flywheel, turret, hood,
      // drive);
      shuttleCommand = TurretCommands.shuttleCommand(flywheel, turret, hood, drive);
      NamedCommands.registerCommand("shoot", shootCommand);
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

    Command pointToHubCommand;

    switch (Constants.controlMode) {
      case JOYSTICK -> {
        if (driveEnabled) {
          driverController.leftBumper().onTrue(toggleIntakeCommand);
          operatorController
              .leftBumper()
              .onTrue(Commands.runOnce(() -> intake.setRelativeVelocity(-0.35), intake));

          // Switch to X pattern when X button is pressed
          driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

          if (flywheelEnabled) {
            // TODO: add flywheel tracking command (indexer at zero speed, flywheel at desired
            // speed)

            driverController.rightBumper().whileTrue(shootCommand);

            operatorController
                .rightBumper()
                .whileTrue(
                    Commands.run(() -> flywheel.runSpindexer(), flywheel)
                        .finallyDo(() -> flywheel.stopSpindexer()));
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

            // operatorController.x().onTrue(intake.deployCommand());
            // operatorController.b().onTrue(intake.retractCommand());
          }
          if (hoodEnabled) {
            // TESTING
            operatorController.x().onTrue(hood.addToPulseCommand(50));
            operatorController.b().onTrue(hood.addToPulseCommand(-50));

            // operatorController.leftBumper().onTrue(hood.setPulseCommand(1000));
            // operatorController.rightBumper().onTrue(hood.setPulseCommand(1500));

            operatorController
                .a()
                .onTrue(Commands.runOnce(() -> System.out.println(hood.getPulse())));

            NamedCommands.registerCommand(
                "turretAngleFar", Commands.runOnce(() -> hood.setPulse(1150), hood));
          } else {
            NamedCommands.registerCommand("turretAngleFar", Commands.none());
          }
          if (turretEnabled) {
            pointToHubCommand = TurretCommands.pointToHub(turret, drive);

            // driverController.rightBumper().whileTrue(pointToHubCommand);

            turret.setDefaultCommand(
                TurretCommands.joystickTurretAngleCommand(
                    turret,
                    drive,
                    () -> operatorController.getRightY(),
                    () -> operatorController.getRightX()));
            operatorController.y().onTrue(turret.resetAngle());
            NamedCommands.registerCommand("point to hub", pointToHubCommand);
            // operatorController
            // .rightTrigger()
            // .onTrue(turret.setTurretAngleCommand(Rotation2d.fromDegrees(90)));
            // operatorController
            // .start()
            // .onTrue(turret.setTurretAngleCommand(Rotation2d.fromDegrees(-90)));
          } else {
            NamedCommands.registerCommand("point to hub", Commands.none());
          }

          // turret and hood
          // UNTESTED
          if (turretEnabled && hoodEnabled) {
            // operatorController.a().whileTrue(ShooterCommands.trackCommand(turret, hood, drive));
          }

          // Reset gyro / odometry

          final Runnable resetGyro =
              () ->
                  drive.setPose(
                      new Pose2d(
                          drive.getPose().getTranslation(),
                          DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                              ? Rotation2d.k180deg
                              : Rotation2d.kZero)); // zero gyro
          driverController.back().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        }
      }
      case KEYBOARD -> {
        // hood.setDefaultCommand(Commands.run(() ->
        // hood.setVelocity(keyboardHID.middleY()),
        // hood));

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

        keyboardHID.v().onTrue(toggleIntakeCommand);

        keyboardHID.m().onTrue(driveAndIntakeCommand);

        keyboardHID.comma().whileTrue(driveAndScoreCommand);

        keyboardHID.period().whileTrue(shuttleCommand);

        // Outpost dump
        keyboardHID.slash().onTrue(Commands.runOnce(() -> {}));
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
    // return AutoBuilder.buildAuto("shoot");
    return autoChooser.getSelected();
  }
}
