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

package org.austincans.lib.util;

public class KalmanFilter1D {
  private double estimatedPoses;
  private double covariance;

  private double processNoiseCovariance; // Process noise covariance
  private double measurementNoisesCovariance; // Measurement noise covariance

  /*
   * How to calculate Measurement Noise
   * 1. Collect static noise of sensor
   * 2. Calculate standard deviation of sensor
   * 3. Covariance = (Std. Deviation) ^2
   *
   * Measurement Noise is == RMS ^ 2
   */

  /*
   * Tune process noise with hopes and dreams
   * or trial and error
   * https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-
   * state-estimation.html
   *
   * Just assume 0
   */

  RootMeanSquare rms = new RootMeanSquare();

  public KalmanFilter1D(
      double initialPose, double initialP, double processNoise, double measurementNoise) {
    this.estimatedPoses = initialPose;
    this.covariance = initialP;
    this.processNoiseCovariance = processNoise;
    this.measurementNoisesCovariance = measurementNoise;
  }

  public KalmanFilter1D(double initialPose, double processNoise, double measurementNoise) {
    this.estimatedPoses = initialPose;
    this.covariance = 0;
    this.processNoiseCovariance = processNoise;
    this.measurementNoisesCovariance = measurementNoise;
  }

  public void predict() {
    this.covariance = this.covariance + this.processNoiseCovariance;
  }

  public void update(double measurement) {
    // Kalman Gain
    double K = this.covariance / (this.covariance + this.measurementNoisesCovariance);

    this.estimatedPoses = this.estimatedPoses + K * (measurement - this.estimatedPoses);
    this.covariance = (1 - K) * this.covariance;
    if (rms.values.size() < 200) {
      rms.storeValue(measurement);
    } else {
      rms.values.remove(rms.values.size() - 1); // Pop
      rms.values.add(0, measurement);
    }
  }

  public double getState() {
    return this.estimatedPoses;
  }
}
