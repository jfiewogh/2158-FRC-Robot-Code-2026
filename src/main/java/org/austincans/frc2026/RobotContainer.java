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
import org.austincans.frc2026.input.DriverMapper;
import org.austincans.frc2026.input.DriverMapperComp;
import org.austincans.frc2026.input.MapperConstants;
import org.austincans.frc2026.input.OperatorMapper;
import org.austincans.frc2026.input.OperatorMapperTest;
import org.austincans.frc2026.subsystems.SubsystemFactory;
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

  // Subsystems
  private final DriveWrapper drive = new DriveWrapper(SubsystemFactory.buildDrive());

  @SuppressWarnings("unused")
  private final Optional<Vision> vision = SubsystemFactory.buildVision(drive.getSubsystem());

  @SuppressWarnings("unused")
  private final Optional<ObjectDetection> objectDetection = SubsystemFactory.buildObjectDetection();

  private final TurretWrapper turret = new TurretWrapper(SubsystemFactory.buildTurret());
  private final HoodWrapper hood = new HoodWrapper(SubsystemFactory.buildHood());
  private final RollerWrapper roller = new RollerWrapper(SubsystemFactory.buildRoller());
  private final DeployWrapper deploy = new DeployWrapper(SubsystemFactory.buildDeploy());
  private final FlywheelWrapper flywheel = new FlywheelWrapper(SubsystemFactory.buildFlywheel());
  private final IndexerWrapper indexer = new IndexerWrapper(SubsystemFactory.buildIndexer());
  private final SpindexerWrapper spindexer =
      new SpindexerWrapper(SubsystemFactory.buildSpindexer());

  @SuppressWarnings("unused")
  private final Optional<Led> led = SubsystemFactory.buildLed();

  private final Superstructure superstructure =
      new Superstructure(drive, turret, hood, roller, deploy, flywheel, indexer, spindexer);

  // Alerts
  private final Alert primaryDisconnected =
      new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected =
      new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);

  // Controllers
  private final DriverMapper driver = new DriverMapperComp();
  private final OperatorMapper operator = new OperatorMapperTest();

  // Dashboard inputs
  private SendableChooser<Command> autoChooser;

  private LoggedTunableNumber flywheelSpeed = new LoggedTunableNumber("FlywheelVelocity", 7500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    registerNamedCommands();

    addAutos();

    configureButtonBindings();

    SmartDashboard.putData("SeqSystemCheck", AdvancedSubsystem.sequentialCheck());
  }

  private void registerNamedCommands() {
    /* Register named commands */
    NamedCommands.registerCommand("Start Intake", superstructure.deployAndRunIntake());
    NamedCommands.registerCommand("End Intake", superstructure.retractAndStopIntake());
    NamedCommands.registerCommand("Score", superstructure.shootToHubCommand());
    NamedCommands.registerCommand("Shuttle", superstructure.shuttleCommand());
    NamedCommands.registerCommand("Still Shoot", superstructure.stillShootCommand());
    NamedCommands.registerCommand("Turret Track Hub", superstructure.turretPointToHub());
  }

  private void addAutos() {
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
    // autoChooser.addOption("Left Auto-One Cycle", AutoBuilder.buildAuto("LeftAutoOneCycle"));
    // autoChooser.addOption("Left Auto with Depot", AutoBuilder.buildAuto("LeftAutoWithDepot"));
    autoChooser.addOption("Mid Auto", AutoBuilder.buildAuto("MidAuto"));
    autoChooser.addOption("Right Auto", AutoBuilder.buildAuto("RightAuto"));

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

    driver.getDeployButton().whileTrue(deploy.deployCommand());
    driver.getRetractButton().whileTrue(deploy.retractCommand());

    driver.getShootButton().whileTrue(superstructure.shootCommand());

    /* Operator */

    turret.setDefaultCommand(
        turret.joystickRobotRelAngleCommand(operator::getRotationX, operator::getRotationY));

    operator.getStaticShooterToggleButton().onTrue(superstructure.staticShooterToggleCommand());

    operator.getTrackingButton().whileTrue(superstructure.turretTrackShuttle());

    operator.getResetTurretAngleButton().onTrue(turret.resetAngleCommand());

    operator.getOuttakeButton().whileTrue(roller.outtakeCommand());

    operator.getSetLowestHoodAngleButton().onTrue(hood.setLowestAngleCommand());

    // Test
    operator.getPointToHubButton().whileTrue(superstructure.turretPointToHub());

    operator.getIncrementHoodAngleButton().onTrue(hood.incrementCommand());
    operator.getDecrementHoodAngleButton().onTrue(hood.decrementCommand());

    operator
        .getRunIndexerAndFlywheelButton()
        .whileTrue(superstructure.runFeederAndFlywheelCommand(flywheelSpeed));
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
