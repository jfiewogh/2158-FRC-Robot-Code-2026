// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.austincans.frc2158.FieldConstants.AprilTagLayoutType;
import org.austincans.frc2158.commands.DriveCommands;
import org.austincans.frc2158.controllers.BlinkinLEDController;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.drive.DriveConstants;
import org.austincans.frc2158.subsystems.drive.GyroIO;
import org.austincans.frc2158.subsystems.drive.GyroIOPigeon2;
import org.austincans.frc2158.subsystems.drive.GyroIORedux;
import org.austincans.frc2158.subsystems.drive.ModuleIO;
import org.austincans.frc2158.subsystems.drive.ModuleIOComp;
import org.austincans.frc2158.subsystems.drive.ModuleIODev;
import org.austincans.frc2158.subsystems.drive.ModuleIOSim;
import org.austincans.frc2158.subsystems.vision.Vision;
import org.austincans.frc2158.util.AllianceFlipUtil;
import org.austincans.frc2158.util.DoublePressTracker;
import org.austincans.frc2158.util.MirrorUtil;
import org.austincans.frc2158.util.OverrideSwitches;
import org.austincans.frc2158.util.TriggerUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@ExtensionMethod({DoublePressTracker.class, TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  @Getter private Drive drive;
  private final BlinkinLEDController leds = BlinkinLEDController.getInstance();

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger robotRelative = overrides.driverSwitch(0);

  private final Trigger aprilTagsReef = overrides.multiDirectionSwitchLeft();
  private final Trigger aprilTagNone = overrides.multiDirectionSwitchRight();
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.kInfo);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);
  private final Alert deadInTheWaterAlert =
      new Alert("Please select an auto routine!!! 😳", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  private Command deadInTheWater = Commands.none();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Vision
  @SuppressWarnings("unused")
  private Vision vision;

  private SwerveDriveSimulation driveSimulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(), // Expansive
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[0]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[1]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[2]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[3]));
          /*
          vision =
              new Vision(
                  drive,
                  new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                  new VisionIOPhotonVision(VisionConstants.camera1Name, drive.getRotation()));
          */
          break;
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIORedux(), // Cheap
                  new ModuleIODev(DriveConstants.moduleConfigsDev[0]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[1]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[2]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[3]));
          // vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        }
        case SIMBOT -> {
          driveSimulation =
              new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
          drive =
              new Drive(
                  new GyroIO() {}, // Virtual
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          /*
                    vision =
                        new Vision(
                            drive,
                            new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name,
                                VisionConstants.robotToCamera0,
                                driveSimulation::getSimulatedDriveTrainPose),
                            new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name,
                                VisionConstants.robotToCamera1,
                                driveSimulation::getSimulatedDriveTrainPose));
          */
          break;
        }
        default -> throw new IllegalArgumentException("Unexpected value: " + Constants.getRobot());
      }
    }

    // No-op implementations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    LoggedDashboardChooser<Boolean> mirror = new LoggedDashboardChooser<>("Processor Side?");
    mirror.addDefaultOption("Yes", false);
    mirror.addOption("No", true);
    LoggedDashboardChooser<Boolean> upInThePush = new LoggedDashboardChooser<>("Up In The Push?");
    upInThePush.addDefaultOption("No", false);
    upInThePush.addOption("Yes", true);
    MirrorUtil.setMirror(mirror::get);

    // Set up characterization routines
    autoChooser.addDefaultOption("Dead In The Water Auto", deadInTheWater);
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

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
    // Drive suppliers
    DoubleSupplier driverX = () -> -driver.getLeftY() - operator.getLeftY();
    DoubleSupplier driverY = () -> -driver.getLeftX() - operator.getLeftX();
    DoubleSupplier driverOmega = () -> -driver.getRightX() - operator.getRightX();

    // Joystick drive command (driver and operator)
    Supplier<Command> joystickDriveCommandFactory =
        () -> DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega, robotRelative);
    drive.setDefaultCommand(joystickDriveCommandFactory.get());

    // ***** DRIVER CONTROLLER *****

    // ***** OPERATOR CONTROLLER *****

    // ***** MISCELANEOUS *****

    // Reset gyro
    var driverStartAndBack = driver.start().and(driver.back());
    var operatorStartAndBack = operator.start().and(operator.back());
    driverStartAndBack
        .or(operatorStartAndBack)
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .withName("Reset Gyro")
                .ignoringDisable(true));

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .beforeStarting(() -> leds.endgameAlert = true)
                .finallyDo(() -> leds.endgameAlert = false)
                .withName("Controller Endgame Alert 1"));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9)
                .beforeStarting(() -> leds.endgameAlert = true)
                .finallyDo(() -> leds.endgameAlert = false)
                .withName("Controller Endgame Alert 2")); // Rumble three times
  }

  // Creates controller rumble command
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(driver.getHID().getPort()));
    operatorDisconnected.set(!DriverStation.isJoystickConnected(operator.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());

    // AprilTag layout alert
    boolean aprilTagAlertActive = getSelectedAprilTagLayout() != FieldConstants.defaultAprilTagType;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-default AprilTag layout in use (" + getSelectedAprilTagLayout().toString() + ").");
    }

    // Auto alert
    deadInTheWaterAlert.set(
        DriverStation.isAutonomous()
            && !DriverStation.isEnabled()
            && autoChooser.get() == deadInTheWater);
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    if (aprilTagsReef.getAsBoolean()) {
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
        return FieldConstants.AprilTagLayoutType.BLUE_REEF;
      } else {
        return FieldConstants.AprilTagLayoutType.RED_REEF;
      }
    } else if (aprilTagNone.getAsBoolean()) {
      return FieldConstants.AprilTagLayoutType.NONE;
    } else {
      return FieldConstants.defaultAprilTagType;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
