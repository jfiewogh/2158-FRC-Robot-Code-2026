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
// Updated on Thursday November 13, 2025 at 14:16:40 CST

package org.austincans.frc2158.util;

import java.util.LinkedList;

/**
 * Implements a simple circular buffer. Can be used for any class.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class CircularBufferGeneric<E> {
  final int mWindowSize;
  final LinkedList<E> mSamples;
  double mSum;

  public CircularBufferGeneric(int window_size) {
    mWindowSize = window_size;
    mSamples = new LinkedList<>();
    mSum = 0.0;
  }

  public void clear() {
    mSamples.clear();
    mSum = 0.0;
  }

  public void addValue(E val) {
    mSamples.addLast(val);
    if (mSamples.size() > mWindowSize) {
      mSamples.removeFirst();
    }
  }

  public int getNumValues() {
    return mSamples.size();
  }

  public boolean isFull() {
    return mWindowSize == mSamples.size();
  }

  public LinkedList<E> getLinkedList() {
    /*
     * NOTE: To get an Array of the specific class type which the instance is using,
     * you have to use this specific code:
     * specificCircularBufferGeneric.getLinkedList().toArray(new ClassThatIWant[specificCircularBufferGeneric
     * .getLinkedList().size()]);
     * The reason is that for some reason an array of a generic class(i.e. E[]) cannot be created because
     * of some archaic data flow ambiguities
     */

    return mSamples;
  }
}
