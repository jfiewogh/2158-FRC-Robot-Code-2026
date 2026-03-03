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

package org.austincans.frc2026.util;

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

  public RootMeanSquare() {}
  ;

  public void storeValue(double val) {
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
