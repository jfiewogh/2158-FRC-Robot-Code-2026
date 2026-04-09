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

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

import org.austincans.frc2026.commands.DriveCommands;
import org.austincans.frc2026.commands.TurretCommands;
import org.austincans.frc2026.commands.shooter.ShooterCommands;
import org.austincans.frc2026.generated.TunerConstants;
import org.austincans.frc2026.input.DriverMapper;
import org.austincans.frc2026.input.DriverMapperComp;
import org.austincans.frc2026.input.MapperConstants;
import org.austincans.frc2026.input.OperatorMapper;
import org.austincans.frc2026.input.OperatorMapperTest;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.drive.GyroIO;
import org.austincans.frc2026.subsystems.drive.GyroIOPigeon2;
import org.austincans.frc2026.subsystems.drive.ModuleIO;
import org.austincans.frc2026.subsystems.drive.ModuleIOTalonFXReal;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIOSparkFlex;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.hood.HoodIO;
import org.austincans.frc2026.subsystems.hood.HoodIOServoLinear;
import org.austincans.frc2026.subsystems.intake.deploy.Deploy;
import org.austincans.frc2026.subsystems.intake.deploy.DeployIO;
import org.austincans.frc2026.subsystems.intake.deploy.DeployIOSparkFlex;
import org.austincans.frc2026.subsystems.intake.roller.Roller;
import org.austincans.frc2026.subsystems.intake.roller.RollerIO;
import org.austincans.frc2026.subsystems.intake.roller.RollerIOSparkFlex;
import org.austincans.frc2026.subsystems.led.Led;
import org.austincans.frc2026.subsystems.led.LedIO;
import org.austincans.frc2026.subsystems.led.LedIOBlinkin;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretIOSparkFlex;
import org.austincans.frc2026.subsystems.vision.ObjectDetectionIOReal;
import org.austincans.frc2026.subsystems.vision.ObjectDetection;
import org.austincans.frc2026.subsystems.vision.ObjectDetectionIO;
import org.austincans.frc2026.subsystems.vision.Vision;
import org.austincans.frc2026.subsystems.vision.VisionConstants;
import org.austincans.frc2026.subsystems.vision.VisionConstants.LimelightConfig;
import org.austincans.frc2026.subsystems.vision.VisionIO;
import org.austincans.frc2026.subsystems.vision.VisionIOLimelight;
import org.austincans.frc2026.util.HubShiftUtil;
import org.austincans.frc2026.util.LoggedTunableNumber;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainerCopy {
  // Subsystem Enabled // only matters for real robot
  private static final boolean driveEnabled = true;
  private static final boolean visionEnabled = true;
  private static final boolean objectDetectionEnabled = false;
  private static final boolean turretEnabled = true;
  private static final boolean hoodEnabled = true;
  private static final boolean rollerEnabled = true;
  private static final boolean deployEnabled = true;
  private static final boolean flywheelEnabled = true;
  private static final boolean ledEnabled = false;

  // Subsystems
  private final Optional<Drive> drive = buildDrive();
  private final Optional<Vision> vision = buildVision();
  private final Optional<ObjectDetection> objectDetection = buildObjectDetection();
  private final Optional<Turret> turret = buildTurret();
  private final Optional<Hood> hood = buildHood();
  private final Optional<Roller> roller = buildRoller();
  private final Optional<Deploy> deploy = buildDeploy();
  private final Optional<Flywheel> flywheel = buildFlywheel();
  private final Optional<Led> led = buildLed();

  // Alerts
  private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected = new Alert("Secondary controller disconnected (port 1).",
      AlertType.kWarning);

  // Controller
  private final DriverMapper driverMapper = new DriverMapperComp();
  private final OperatorMapper operatorMapper = new OperatorMapperTest();

  // Dashboard inputs
  private SendableChooser<Command> autoChooser;

  private LoggedTunableNumber flywheelSpeed = new LoggedTunableNumber("FlywheelVelocity", 7500.0);

  private Optional<Drive> buildDrive() {
    if (!driveEnabled) {
      return Optional.empty();
    }
    Drive drive;
    if (Constants.currentMode == Constants.Mode.REAL) {
      drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
          new ModuleIOTalonFXReal(TunerConstants.FrontRight),
          new ModuleIOTalonFXReal(TunerConstants.BackLeft),
          new ModuleIOTalonFXReal(TunerConstants.BackRight),
          (pose) -> {
          });
    } else {
      drive = new Drive(
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
    }
    return Optional.ofNullable(drive);
  }

  private Optional<Vision> buildVision() {
    if (!driveEnabled) {
      return Optional.empty();
    }
    if (!visionEnabled) {
      return Optional.empty();
    }
    VisionIO[] cameras;
    if (Constants.currentMode == Constants.Mode.REAL) {
      cameras = new VisionIO[] {
        new VisionIOLimelight(VisionConstants.camera0Name, drive.get()::getRotation),
        new VisionIOLimelight(VisionConstants.camera1Name, drive.get()::getRotation)
      };
    } else {
      cameras = new VisionIO[] {
        new VisionIO() {},
        new VisionIO() {}
      };
    }
    return Optional.ofNullable(new Vision(drive.get(), cameras));
  }

  private Optional<Turret> buildTurret() {
    if (!turretEnabled) {
      return Optional.empty();
    }
    Turret turret = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      turret = new Turret(new TurretIOSparkFlex());
    }
    return Optional.ofNullable(turret);
  }

  private Optional<Hood> buildHood() {
    if (!hoodEnabled) {
      return Optional.empty();
    }
    Hood hood = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      hood = new Hood(new HoodIOServoLinear());
    }
    return Optional.ofNullable(hood);
  }

  private Optional<Roller> buildRoller() {
    if (!rollerEnabled) {
      return Optional.empty();
    }
    Roller roller = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      roller = new Roller(new RollerIOSparkFlex());
    }
    return Optional.ofNullable(roller);
  }

  private Optional<Deploy> buildDeploy() {
    if (!deployEnabled) {
      return Optional.empty();
    }
    Deploy deploy = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      deploy = new Deploy(new DeployIOSparkFlex());
    }
    return Optional.ofNullable(deploy);
  }

  private Optional<Flywheel> buildFlywheel() {
    if (!flywheelEnabled) {
      return Optional.empty();
    }
    Flywheel flywheel = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      flywheel = new Flywheel(new FlywheelIOSparkFlex());
    }
    return Optional.ofNullable(flywheel);
  }

  private Optional<ObjectDetection> buildObjectDetection() {
    if (!objectDetectionEnabled) {
      return Optional.empty();
    }
    ObjectDetection objectDetection = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      objectDetection = new ObjectDetection(new ObjectDetectionIOReal(
        "limelight",
        ObjectDetection.GamePieceType.FUEL,
        LimelightConfig.LIMELIGHT_3G));
    }
    return Optional.ofNullable(objectDetection);
  }

  private Optional<Led> buildLed() {
    if (!ledEnabled) {
      return Optional.empty();
    }
    LedIO io = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      io = new LedIOBlinkin();
    }
    return Optional.ofNullable(new Led(io));
  }

  private boolean allSubsystemsPresent(Optional<?>... subsystems) {
    return Arrays.stream(subsystems).allMatch(s -> s.isPresent());
  }

  private Command getCommand(Supplier<Command> commandSupplier, Optional<?>... subsystems) {
    return allSubsystemsPresent(subsystems) ? commandSupplier.get() : Commands.none();
  }

  private void registerNamedCommand(String name, Supplier<Command> commandSupplier, Optional<?>... subsystems) {
    NamedCommands.registerCommand(name, allSubsystemsPresent(subsystems) ? commandSupplier.get() : Commands.none());
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainerCopy() {
    registerNamedCommands();

    addAutos();

    configureButtonBindings();

    SmartDashboard.putData("SeqSystemCheck", AdvancedSubsystem.sequentialCheck());
  }

  private void registerNamedCommands() {
    /* Register named commands */
    registerNamedCommand("Run Intake",
        () -> Commands.startEnd(roller.get()::runIntake, roller.get()::stop, roller.get()),
        roller);
    registerNamedCommand("Score",
        () -> ShooterCommands.shootToHubCommand(drive.get(), flywheel.get(), hood.get(), turret.get()),
        drive, flywheel, hood, turret);
    registerNamedCommand("Shuttle",
        () -> ShooterCommands.shuttleCommand(drive.get(), flywheel.get(), hood.get(), turret.get()),
        drive, flywheel, hood, turret);
    registerNamedCommand("turretAngleFar",
        () -> Commands.runOnce(() -> hood.get().setPulse(1150), hood.get()),
        hood);
    registerNamedCommand("point to hub",
        () -> TurretCommands.pointToHub(turret.get(), drive.get()),
        turret, drive);
  }

  private void addAutos() {
    // Set up auto routines
    autoChooser = new SendableChooser<Command>();

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive.get()));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive.get()));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.get().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Angular)",
        drive.get().sysIdQuasistaticAngular(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.get().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.get().sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynam ic Angular)",
        drive.get().sysIdDynamicAngular(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.get().sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    /* Default Commands */
    drive.ifPresent((d) -> d.setDefaultCommand(getCommand(() -> DriveCommands.joystickDriveCommand(
        drive.get(),
        driverMapper::getTranslationX,
        driverMapper::getTranslationY,
        driverMapper::getRotationX,
        driverMapper::getRotationY), drive)));

    turret.ifPresent((t) -> t.setDefaultCommand(getCommand(() -> TurretCommands.joystickTurretAngleCommand(
        turret.get(),
        operatorMapper::getRotationX,
        operatorMapper::getRotationY), turret)));

    /* Driver */
    driverMapper.getRollerToggleButton().onTrue(getCommand(roller.get()::toggleCommand, roller));
    driverMapper.getTurnModeToggleButton().onTrue(DriveCommands.setTurnMode());
    driverMapper.getShootButton()
      .onTrue(
          getCommand(
              () -> ShooterCommands.shootCommand(flywheel.get(), turret.get(), hood.get(), drive.get()),
            drive, flywheel, hood, turret));
    
    if (driverMapper instanceof DriverMapperComp) {
      DriverMapperComp driverMapperComp = (DriverMapperComp) driverMapper;

      driverMapperComp.getDeployButton()
        .onTrue(getCommand(deploy.get()::deployCommand, deploy))
        .onFalse(getCommand(deploy.get()::stopCommand, deploy));

      driverMapperComp.getRetractButton()
        .onTrue(getCommand(deploy.get()::retractCommand, deploy))
        .onFalse(getCommand(deploy.get()::stopCommand, deploy));

      driverMapperComp.getStopWithXButton()
        .onTrue(
            getCommand(
              () -> Commands.runOnce(
                drive.get()::stopWithX, drive.get()), 
                drive));

      // Reset gyro / odometry
      Runnable resetGyro = () -> drive.get().setPose(
              new Pose2d(
                  drive.get().getPose().getTranslation(),
                  DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                      ? Rotation2d.k180deg
                      : Rotation2d.kZero));
      driverMapperComp.getResetPoseButton()
        .onTrue(
            getCommand(
              () -> Commands.runOnce(
                resetGyro, drive.get()).ignoringDisable(true), drive));
    }

    /* Operator */
    operatorMapper.getOuttakeButton().onTrue(getCommand(roller.get()::runOuttakeCommand, roller));
    operatorMapper.getStaticShooterToggleButton()
      .onTrue(Commands.runOnce(() -> ShooterCommands.staticShooter = !ShooterCommands.staticShooter));

    if (operatorMapper instanceof OperatorMapperTest) {
      OperatorMapperTest operatorMapperTest = (OperatorMapperTest) operatorMapper;

      operatorMapperTest.getPointToHubButton()
        .whileTrue(
            getCommand(
              () -> Commands.run(
                () -> flywheel.get()
                  .runIndexerAndFlywheel(flywheelSpeed.getAsDouble()), flywheel.get()),
              flywheel));

      operatorMapperTest.getRunIndexerAndFlywheelButton()
        .whileTrue(
            getCommand(
              () -> Commands.run(
                () -> flywheel.get()
                  .runIndexerAndFlywheel(flywheelSpeed.getAsDouble()), flywheel.get())
                  .finallyDo(flywheel.get()::stopMotors), 
              flywheel));
    }
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
    primaryDisconnected.set(!DriverStation.isJoystickConnected(MapperConstants.driverPort));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(MapperConstants.operatorPort));
  }
}
