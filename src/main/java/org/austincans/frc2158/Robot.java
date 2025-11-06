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

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.austincans.frc2158.Constants.RobotType;
import org.austincans.frc2158.controllers.REVDigitController;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers;
import org.austincans.frc2158.util.CanivoreReader;
import org.austincans.frc2158.util.DummyLogReceiver;
import org.austincans.frc2158.util.LoggedTracer;
import org.austincans.frc2158.util.NTClientLogger;
import org.austincans.frc2158.util.PhoenixUtil;
import org.austincans.frc2158.util.SystemTimeValidReader;
import org.austincans.frc2158.util.VirtualSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

@java.lang.SuppressWarnings({"java:S1192", "java:S106"})

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private static final double loopOverrunWarningTimeout = 0.2;
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double canivoreErrorTimeThreshold = 0.5;
  private static final double lowBatteryVoltage = 10.0;
  private static final double lowBatteryDisabledTime = 1.5;
  private static final double lowBatteryMinCycleCount = 10;
  private static int lowBatteryCycleCount = 0;

  private double autoStart;
  private boolean autoMessagePrinted;
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private final Timer disabledTimer = new Timer();
  private final CanivoreReader canivoreReader = new CanivoreReader("*");

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert canivoreErrorAlert =
      new Alert("CANivore errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);
  private final Alert jitAlert =
      new Alert("Please wait to enable, JITing in progress.", AlertType.kWarning);

  // Camera
  BooleanPublisher cameraServer, frontLeft, frontRight, rearLeft, rearRight, llVision, pvVision;
  // Lidar
  BooleanPublisher lidarServer, frStatus, flStatus, rrStatus, rlStatus;
  DoublePublisher frDistance, flDistance, rrDistance, rlDistance;
  DoublePublisher frSignalStrength, flSignalStrength, rrSignalStrength, rlSignalStrength;
  DoublePublisher frChipTemp, flChipTemp, rrChipTemp, rlChipTemp;
  // Fan Control
  DoublePublisher radioTempC, radioTempF, fanSpeed, fan1RPM, fan2RPM;
  BooleanPublisher fanCtrlServer, fanStatus;
  // Control
  DoublePublisher initTime, cpuTemp, loadAvg, diskUsage;
  BooleanPublisher ctrlStatus, pigpiod, audio;
  // UPS
  StringPublisher upsId, upsVersion;
  DoublePublisher outputVoltage,
      inputVoltage,
      batteryVoltage,
      mcuVoltage,
      outputCurrent,
      inputCurrent,
      batteryCurrent,
      temperature,
      inputPower,
      outputPower;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Add dummy log receiver to adjust thread priority
    Logger.addDataReceiver(new DummyLogReceiver());

    // Start AdvantageKit logger
    Logger.start();

    // Disable automatic Hoot logging
    SignalLogger.enableAutoLogging(false);

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

    // Start system time valid reader
    SystemTimeValidReader.start();

    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Silence Rotation2d warnings
    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    // Configure brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Configure DriverStation for sim
    if (Constants.getRobot() == RobotType.SIMBOT) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Threads.setCurrentThreadPriority(true, 1);
    createCameraEntries();
    createLidarEntries();
    createFanCtrlEntries();
    createCtrlEntries();
    createUPSEntries();
  }

  public void createCameraEntries() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("frc2158/camera");
    frontRight = table.getBooleanTopic("frontright").publish();
    frontRight.set(true);
    frontLeft = table.getBooleanTopic("frontleft").publish();
    frontLeft.set(true);
    rearRight = table.getBooleanTopic("rearright").publish();
    rearRight.set(true);
    rearLeft = table.getBooleanTopic("rearleft").publish();
    rearLeft.set(true);
    llVision = table.getBooleanTopic("llvision").publish();
    llVision.set(true);
    pvVision = table.getBooleanTopic("pvvision").publish();
    pvVision.set(true);

    cameraServer = table.getBooleanTopic("server").publish();
    cameraServer.set(true);
  }

  public void createLidarEntries() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("frc2158/lidar");
    // Front Left
    flStatus = table.getBooleanTopic("fllidarversion").publish();
    flStatus.set(false);
    flDistance = table.getDoubleTopic("fldistance").publish();
    flDistance.set(12);
    flSignalStrength = table.getDoubleTopic("flsignalstrength").publish();
    flSignalStrength.set(500);
    flChipTemp = table.getDoubleTopic("flchiptemp").publish();
    flChipTemp.set(26);
    // Front Right
    frStatus = table.getBooleanTopic("frlidarversion").publish(); // Ready or Failure
    frStatus.set(true);
    frDistance = table.getDoubleTopic("frdistance").publish(); //   distance to target in cm
    frDistance.set(12);
    frSignalStrength =
        table
            .getDoubleTopic("frsignalstrength")
            .publish(); // Signal strength (Amp): Distance value is unreliable when receiving signal
    // is overexposure (Amp = 0xFFFF) or too low (Amp < 100 as 14)
    frSignalStrength.set(500);
    frChipTemp =
        table
            .getDoubleTopic("frchiptemp")
            .publish(); // Chip Temperature (Temp): Celsius degree = Temp/8 - 256℃
    frChipTemp.set(26);
    // Rear Left
    rlStatus = table.getBooleanTopic("rllidarversion").publish();
    rlStatus.set(true);
    rlDistance = table.getDoubleTopic("rldistance").publish();
    rlDistance.set(28);
    rlSignalStrength = table.getDoubleTopic("rlsignalstrength").publish();
    rlSignalStrength.set(700);
    rlChipTemp = table.getDoubleTopic("rlchiptemp").publish();
    rlChipTemp.set(17);
    // Rear Right
    rrStatus = table.getBooleanTopic("rrlidarversion").publish();
    rrStatus.set(false);
    rrDistance = table.getDoubleTopic("rrdistance").publish();
    rrDistance.set(12);
    rrSignalStrength = table.getDoubleTopic("rrsignalstrength").publish();
    rrSignalStrength.set(500);
    rrChipTemp = table.getDoubleTopic("rrchiptemp").publish();
    rrChipTemp.set(26);

    lidarServer = table.getBooleanTopic("server").publish();
    lidarServer.set(true);
  }

  public void createFanCtrlEntries() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("frc2158/fanctrl");
    radioTempC = table.getDoubleTopic("radiotempc").publish();
    radioTempC.set(25);
    radioTempF = table.getDoubleTopic("radiotempf").publish();
    radioTempF.set(77);
    fanStatus = table.getBooleanTopic("fanstatus").publish();
    fanStatus.set(false);
    fanSpeed = table.getDoubleTopic("fanspeed").publish();
    fanSpeed.set(0);
    fan1RPM = table.getDoubleTopic("fan1rpm").publish();
    fan1RPM.set(0);
    fan2RPM = table.getDoubleTopic("fan2rpm").publish();
    fan2RPM.set(0);

    fanCtrlServer = table.getBooleanTopic("server").publish();
    fanCtrlServer.set(true);
  }

  public void createCtrlEntries() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("frc2158/ctrl");
    initTime = table.getDoubleTopic("inittime").publish();
    initTime.set(481921.95);
    ctrlStatus = table.getBooleanTopic("status").publish();
    ctrlStatus.set(true);
    pigpiod = table.getBooleanTopic("pigpiod").publish();
    pigpiod.set(true);
    cpuTemp = table.getDoubleTopic("cputemp").publish();
    cpuTemp.set(43);
    loadAvg = table.getDoubleTopic("loadavg").publish();
    loadAvg.set(0.6);
    diskUsage = table.getDoubleTopic("diskusage").publish();
    diskUsage.set(13);
    audio = table.getBooleanTopic("audio").publish();
    audio.set(false);
  }

  public void createUPSEntries() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("frc2158/UPS");
    upsId = table.getStringTopic("ID").publish();
    upsId.set("0x46");
    upsVersion = table.getStringTopic("version").publish();
    upsVersion.set("1");
    outputVoltage = table.getDoubleTopic("outputvoltage").publish();
    outputVoltage.set(5.227);
    inputVoltage = table.getDoubleTopic("inputvoltage").publish();
    inputVoltage.set(12.218);
    batteryVoltage = table.getDoubleTopic("batteryvoltage").publish();
    batteryVoltage.set(8.190);
    mcuVoltage = table.getDoubleTopic("mcuvoltage").publish();
    mcuVoltage.set(3.313);
    outputCurrent = table.getDoubleTopic("outputcurrent").publish();
    outputCurrent.set(5.07);
    inputCurrent = table.getDoubleTopic("inputcurrent").publish();
    inputCurrent.set(2.67);
    batteryCurrent = table.getDoubleTopic("batterycurrent").publish();
    batteryCurrent.set(4);
    temperature = table.getDoubleTopic("temperature").publish();
    temperature.set(34);
    inputPower = table.getDoubleTopic("inputpower").publish();
    inputPower.set(3.26);
    outputPower = table.getDoubleTopic("outputpower").publish();
    outputPower.set(2.65);
  }

  public void robotInit() {
    // Initialize the REVDigit MXP Board
    // This is used to display the team number on the REVDigit
    REVDigitController rdc = REVDigitController.getInstance();
    rdc.writeDigits("2158"); // Display 2158 on the REVDigit

    // Output 2158 logo
    String ausTINCANs =
        "                _____ ___ _   _    ____    _    _   _\r\n"
            + "  __ _ _   _ __|_   _|_ _| \\ | |  / ___|  / \\  | \\ | |___\r\n"
            + " / _` | | | / __|| |  | ||  \\| | | |     / _ \\ |  \\| / __|\r\n"
            + "| (_| | |_| \\__ \\| |  | || |\\  | | |___ / ___ \\| |\\  \\__ \\\r\n"
            + " \\__,_|\\__,_|___/|_| |___|_| \\_|  \\____/_/   \\_\\_| \\_|___/\r\n";
    System.out.println(ausTINCANs);
    System.out.println("Welcome to the 2025 ausTIN CANs Robot Code!");
    System.out.println(
        "Running "
            + BuildConstants.MAVEN_NAME
            + " Version "
            + BuildConstants.VERSION
            + " ("
            + BuildConstants.BUILD_DATE
            + ")\n");
  }

  /** This function is called periodically during all modes. */
  @SuppressWarnings("unused")
  @Override
  public void robotPeriodic() {
    // Refresh all Phoenix signals
    LoggedTracer.reset();
    PhoenixUtil.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    // Run virtual subsystems
    VirtualSubsystem.periodicAll();

    // Run command scheduler
    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");

    // Print auto duration
    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    // Robot container periodic methods
    // robotContainer.updateAlerts();
    // robotContainer.updateDashboardOutputs();

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Log CANivore status
    if (Constants.getMode() == Constants.Mode.REAL) {
      var canivoreStatus = canivoreReader.getStatus();
      if (canivoreStatus.isPresent()) {
        Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.get().Status.getName());
        Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.get().BusUtilization);
        Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.get().BusOffCount);
        Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.get().TxFullCount);
        Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.get().REC);
        Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.get().TEC);
        if (!canivoreStatus.get().Status.isOK()
            || canStatus.transmitErrorCount > 0
            || canStatus.receiveErrorCount > 0) {
          canivoreErrorTimer.restart();
        }
      }
      canivoreErrorAlert.set(
          !canivoreErrorTimer.hasElapsed(canivoreErrorTimeThreshold)
              && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

      /*
       * This example of adding Limelight is very simple and may not be sufficient for on-field use.
       * Users typically need to provide a standard deviation that scales with the distance to target
       * and changes with number of tags available.
       *
       * This example is sufficient to show that vision integration is possible, though exact implementation
       * of how to use vision should be tuned per-robot and to the team's specification.
       */
      boolean kUseLimelight = true;
      if (kUseLimelight) {
        LimelightHelpers.PoseEstimate llMeasurement;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
          llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        } else {
          llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
        }
        /*
        if (llMeasurement != null) {
          robotContainer
              .getDrive()
              .addVisionMeasurement(
                  llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
        }
        */
      }
    }

    // Log NT client list
    NTClientLogger.log();

    // Low battery alert
    lowBatteryCycleCount += 1;
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)
        && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
      lowBatteryAlert.set(true);
      // Leds.getInstance().lowBatteryAlert = true;
    }

    // JIT alert
    jitAlert.set(isJITing());

    // Record cycle time
    LoggedTracer.record("RobotPeriodic");

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** Returns whether we should wait to enable because JIT optimizations are in progress. */
  public static boolean isJITing() {
    return Timer.getTimestamp() < 45.0;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStart = Timer.getTimestamp();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    AIRobotInSimulation.startOpponentRobotSimulations();

    SimulatedArena.getInstance().resetFieldForAuto();

    SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)));
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();

    // Publish to telemetry using AdvantageKit
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
  }
}
