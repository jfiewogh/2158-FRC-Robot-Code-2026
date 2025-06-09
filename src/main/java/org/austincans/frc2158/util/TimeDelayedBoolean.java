// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * This class contains a boolean value and a timer. It can set its boolean value and return whether
 * the timer is within a set timeout. This returns true if the stored value is true and the timeout
 * has expired.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class TimeDelayedBoolean {
  private Timer t = new Timer();
  private boolean m_old = false;

  public boolean update(boolean value, double timeout) {
    if (!m_old && value) {
      t.reset();
      t.start();
    }
    m_old = value;
    return value && t.get() >= timeout;
  }
}
