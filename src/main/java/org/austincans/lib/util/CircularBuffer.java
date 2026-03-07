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

import java.util.LinkedList;

/**
 * Implements a simple circular buffer.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class CircularBuffer {
  final int mWindowSize;
  final LinkedList<Double> mSamples;
  double mSum;

  public CircularBuffer(int window_size) {
    mWindowSize = window_size;
    mSamples = new LinkedList<>();
    mSum = 0.0;
  }

  public void clear() {
    mSamples.clear();
    mSum = 0.0;
  }

  public double getAverage() {
    if (mSamples.isEmpty()) return 0.0;
    return mSum / mSamples.size();
  }

  public void recomputeAverage() {
    // Reset any accumulation drift.
    mSum = 0.0;
    if (mSamples.isEmpty()) return;
    for (Double val : mSamples) {
      mSum += val;
    }
    mSum /= mWindowSize;
  }

  public void addValue(double val) {
    mSamples.addLast(val);
    mSum += val;
    if (mSamples.size() > mWindowSize) {
      mSum -= mSamples.removeFirst();
    }
  }

  public int getNumValues() {
    return mSamples.size();
  }

  public boolean isFull() {
    return mWindowSize == mSamples.size();
  }
}
