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
// Updated on Wednesday November 12, 2025 at 18:18:23 CST

package org.austincans.frc2158.util;

import java.util.ArrayList;
import java.util.DoubleSummaryStatistics;
import java.util.List;

/**
 * Finds the stats (mean, standard deviation, etc.) of a list
 *
 * <p>Example use case: finding out how long a planner takes from the average of 100 tries
 *
 * @author Austin Cans Robotics Team 2158
 */
public class StatFinder {
  private final List<Double> numbers = new ArrayList<>();
  private final int numberToIgnore;
  private int numberIgnored = 0;

  private boolean stopped = false;

  /**
   * Creates a new StatFinder
   *
   * @param numberToIgnore the number of entries to ignore before logging (like burning the top card
   *     of a deck before dealing)
   */
  public StatFinder(int numberToIgnore) {
    this.numberToIgnore = numberToIgnore;
  }

  public boolean add(double number) {
    if (stopped) {
      return false;
    }

    if (numberIgnored < numberToIgnore) {
      numberIgnored++;
      return false;
    }

    numbers.add(number);
    return true;
  }

  public boolean addAndPrint(double number) {
    boolean success = add(number);

    if (success) {
      System.out.println("added: " + number);
    }

    return success;
  }

  public DoubleSummaryStatistics getStats() {
    return numbers.stream().mapToDouble(x -> x).summaryStatistics();
  }

  public double getMean() {
    return getStats().getAverage();
  }

  public double getStandardDeviation() {
    double mean = getMean();
    return Math.sqrt(
        numbers.stream().mapToDouble(x -> Math.pow(x - mean, 2)).sum() / (getSize() - 1));
  }

  public double getSize() {
    return numbers.size();
  }

  public void printStats() {
    System.out.println("mean: " + getMean());
    System.out.println("standard deviation: " + getStandardDeviation());
    System.out.println("min: " + getStats().getMin());
    System.out.println("max: " + getStats().getMax());
    System.out.println("size: " + getSize());
  }

  public void stop() {
    stopped = true;
  }

  public void stopAndPrint() {
    if (stopped) {
      return;
    }

    stop();
    printStats();
  }
}
