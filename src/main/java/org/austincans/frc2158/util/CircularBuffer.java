// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

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
