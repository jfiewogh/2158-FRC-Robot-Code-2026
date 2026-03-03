package org.austincans.frc2158.util;

import java.lang.Math;
import java.util.ArrayList;

public class RootMeanSquare {
    public ArrayList<Double> values;
    public static double calculateRMS(double[] numbers) {
        if (numbers == null || numbers.length == 0) {
            return 0.0;
        }

        double sumOfSquares = 0.0;
        for (double value : numbers) {
            sumOfSquares += value * value;
        }

        double mean = sumOfSquares / numbers.length;
        return Math.sqrt(mean);
    }
    public RootMeanSquare(){};

    public void storeValue(double val){
        values.add(val);
    }

    public double calculateRMS() {
        if (values == null || values.size() == 0) {
            return 0.0;
        }

        double sumOfSquares = 0.0;
        for (double value : values) {
            sumOfSquares += value * value;
        }

        double mean = sumOfSquares / values.size();
        return Math.sqrt(mean);
    }
}
