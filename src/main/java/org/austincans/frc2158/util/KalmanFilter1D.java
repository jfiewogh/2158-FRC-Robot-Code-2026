package org.austincans.frc2158.util;

public class KalmanFilter1D {
    private double estimatedPoses; // Estimated pose
    private double covariance; 
    
    private double processsNoiseCovariance; // Process noise covariance
    private double measurementNoisesCovariance; // Measurement noise covariance
    /*
     * How to calculate Measurement Noise 
     * 1. Collect static noise of sensor
     * 2. Calculate standard deviation of sensor
     * 3. Covariance = (Std. Deviation) ^2
     */
    
    public KalmanFilter1D(double initialPose, double initialP, double processNoise, double measurementNoise) {
        this.estimatedPoses = initialPose;
        this.covariance = initialP;
        this.processsNoiseCovariance = processNoise;
        this.measurementNoisesCovariance = measurementNoise;
    }

    public KalmanFilter1D(double initialPose, double processNoise, double measurementNoise) {
        this.estimatedPoses = initialPose;
        this.covariance = 0;
        this.processsNoiseCovariance = processNoise;
        this.measurementNoisesCovariance = measurementNoise;
    }
    
    public void predict() {
        this.covariance = this.covariance + this.processsNoiseCovariance;
    }
    
    public void update(double measurement) {
        // Kalman Gain
        double K = this.covariance / (this.covariance + this.measurementNoisesCovariance);
        
        this.estimatedPoses = this.estimatedPoses + K * (measurement - this.estimatedPoses);
        this.covariance = (1 - K) * this.covariance;
    }
    
    public double getState() {
        return this.estimatedPoses;
    }
}
