// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.austincans.frc2158.util;

public class AutoPID {
  private static double target = 0;
  private static double loopInterval = 0; // Secs
  private static long startTime = 0;
  private static double minOutput = 0;
  private static double maxOutput = 0;
  private static double cycles = 10;

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
  public static double tunePID(float input) {
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

  public static boolean isFinished() {
    return (i >= cycles);
  }

  // return number of tuning cycle
  public static int getCycle() {
    return i;
  }
}
/*
 * Steps to Use
 * 1. setTargetInputValue (the setpoint you will use in the future)
 * 2. setLoopInterval (the *realistic* time you expect to reach the setpoint)
 * 3. setOutputRange (the range of value for your system [0, 1], [0, 255])
 * 4. setZieglerMode (noOverShoot best)
 * 5. start with current time in microseconds
 */

/*
 * setTargetInputValue(100);
 * setLoopInterval((long) 500000); // 500ms to micro
 * setOutputRange(0, 255);
 * setZieglerMode(ZieglerMode.noOverShoot);
 * 
 * // must call immediately before loop
 * // Must be called with the current time in microseconds
 * initLoopStart(RobotController.getFPGATime());
 * 
 * long microseconds;
 * while (!isFinished()) {
 * long prevMicroseconds = microseconds;
 * microseconds = RobotController.getFPGATime();
 * 
 * // (temperature, encoder position, velocity, etc)
 * double input = ReplaceWithClosedLoopInputValues;
 * double output = tunePID(input, microseconds);
 * ReplaceWithPIDOutputIntoClosedSystem(output);
 * 
 * // This loop must run at the same speed as the PID control loop
 * while (RobotController.getFPGATime() - microseconds < loopInterval){
 * Timer.delay(0); // Replace with average loop times for PID system
 * }
 * }
 * 
 * // Turn the output off here.
 * ReplaceWithPIDOutputIntoClosedSystem(0);
 * 
 * // Get PID values
 * double kp = getKp();
 * double ki = getKi();
 * double kd = getKd();
 */