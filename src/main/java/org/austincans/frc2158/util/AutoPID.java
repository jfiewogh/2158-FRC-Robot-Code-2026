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
// Updated on Thursday February 05, 2026 at 19:51:21 CST

package org.austincans.frc2158.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoPID {
  private static double target = 0;
  private static double loopInterval = 0; // Secs
  private static long startTime = 0;
  private static double minOutput = 0;
  private static double maxOutput = 0;
  private static double cycles = 20;

  private static int i;
  private static boolean output;
  private static double outputValue;
  private static long time1, time2, timeAtHigherOscillation, timeAtLowerOscillation;
  private static double max, min;
  private static double kP, kI, kD;
  private static double pAverage, iAverage, dAverage;

  public static enum ZieglerMethod {
    noOverShoot, // Slowest, Safest
    lowOverShoot,
    classicPID, // Fastest high overshoot
  }

  private static ZieglerMethod ZieglerMode = ZieglerMethod.noOverShoot;

  public static void setTargetInputValue(double value) {
    target = value;
  }

  public static void setOutputRange(double minVal, double maxVal) {
    min = minVal;
    max = maxVal;
  }

  public static void setZieglerMode(ZieglerMethod mode) {
    ZieglerMode = mode;
  }

  // The amount of time in microseconds this PID loop is expected to run at in
  // practice
  public static void setLoopInterval(long microSecondInterval) {
    loopInterval = microSecondInterval;
  }

  public static void initLoopStart(long time) {
    startTime = time;
    kP = 0;
    kI = 0;
    kD = 0;
    pAverage = 0;
    iAverage = 0;
    dAverage = 0;
    i = 0;
  }

  // one cycle of the tuning loop
  public static double tunePID(double input) {
    // http://www.processcontrolstuff.net/wp-content/uploads/2015/02/relay_autot-2.pdf
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    // https://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf

    max = (max > input) ? max : input;
    min = (min < input) ? min : input;

    if (output && input > target) {
      output = false;
      outputValue = minOutput;
      time1 = startTime;
      timeAtHigherOscillation = time1 - time2;
      max = target;
    }

    if (!output && input < target) {
      output = true;
      outputValue = maxOutput;
      time2 = startTime;
      timeAtLowerOscillation = time2 - time1;

      // Calculate Ku (ultimate gain)
      double Ku = (4.0 * ((maxOutput - minOutput) / 2.0)) / (Math.PI * (max - min) / 2.0);

      // Calculate Tu (period of output oscillations)
      float Tu = timeAtLowerOscillation + timeAtHigherOscillation;

      double KpConstant, TiConstant, TdConstant;
      if (ZieglerMode == ZieglerMethod.lowOverShoot) {
        KpConstant = 0.33;
        TiConstant = 0.5;
        TdConstant = 0.33;
      } else if (ZieglerMode == ZieglerMethod.noOverShoot) {
        KpConstant = 0.2;
        TiConstant = 0.5;
        TdConstant = 0.33;
      } else if (ZieglerMode == ZieglerMethod.classicPID) {
        KpConstant = 0.6;
        TiConstant = 0.5;
        TdConstant = 0.125;
      } else { // Default to No Overshoot mode as it is the safest
        KpConstant = 0.2;
        TiConstant = 0.5;
        TdConstant = 0.33;
      }

      // Calculate gains
      // REF: Ziegler Method
      kP = KpConstant * Ku;
      kI = (kP / (TiConstant * Tu)) * loopInterval;
      kD = (TdConstant * kP * Tu) / loopInterval;

      // Average all gains
      if (i > 1) {
        pAverage += kP;
        iAverage += kI;
        dAverage += kD;
      }

      // Reset minimum
      min = target;

      // Increment cycle count
      i++;
    }

    // If loop is done, disable output and calculate averages
    if (i >= cycles) {
      output = false;
      outputValue = minOutput;
      kP = pAverage / (i - 1);
      kI = iAverage / (i - 1);
      kD = dAverage / (i - 1);
    }

    return outputValue;
  }

  public static boolean tunerFinished() {
    return (i >= cycles);
  }

  // return number of tuning cycle
  public static int getCycle() {
    return i;
  }

  public static Command runTuner(
      double setpointValue,
      long interval,
      double outputMin,
      double outputMax,
      ZieglerMethod mode,
      Supplier<Double> encoderVals,
      Consumer<Double> pidOutput,
      Runnable stopMotors) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  setTargetInputValue(setpointValue);
                }),
            Commands.runOnce(
                () -> {
                  setLoopInterval(interval);
                }),
            Commands.runOnce(
                () -> {
                  setOutputRange(outputMin, outputMax);
                }),
            Commands.runOnce(
                () -> {
                  setZieglerMode(mode);
                }),
            Commands.runOnce(
                () -> {
                  initLoopStart(RobotController.getFPGATime());
                }),
            new Command() {
              private long microseconds;

              @Override
              public void execute() {
                microseconds = RobotController.getFPGATime();

                // (temperature, encoder position, velocity, etc)
                double input = encoderVals.get();
                double output = tunePID(input);
                pidOutput.accept(output);

                // This loop must run at the same speed as the PID control loop
                while (RobotController.getFPGATime() - microseconds < loopInterval) {
                  Timer.delay(loopInterval / 2); // Replace with average loop times for PID system
                }
              }
              ;

              @Override
              public boolean isFinished() {
                return tunerFinished();
              }
            })
        .andThen(
            () -> {
              stopMotors.run();
            })
        .andThen(
            () -> {
              System.out.println("P: " + kP);
              System.out.println("I: " + kI);
              System.out.println("D: " + kD);
            });
  }
}
