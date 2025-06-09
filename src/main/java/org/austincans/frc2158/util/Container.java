// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

/**
 * Stores mutable a value
 *
 * @author Austin Cans Robotics Team 2158
 */
public class Container<T> {

  /** The value being stored */
  public T value;

  /** Create new container with initial value. */
  public Container(T value) {
    this.value = value;
  }
}
