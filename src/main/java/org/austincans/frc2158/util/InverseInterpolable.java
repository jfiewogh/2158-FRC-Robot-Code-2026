// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

/**
 * InverseInterpolable is an interface used by an Interpolating Tree as the Key type. Given two
 * endpoint keys and a third query key, an InverseInterpolable object can calculate the
 * interpolation parameter of the query key on the interval [0, 1].
 *
 * @param <T> The Type of InverseInterpolable
 * @see InterpolatingTreeMap
 * @author Austin Cans Robotics Team 2158
 */
public interface InverseInterpolable<T> {
  /**
   * Given this point (lower), a query point (query), and an upper point (upper), estimate how far
   * (on [0, 1]) between 'lower' and 'upper' the query point lies.
   *
   * @param upper
   * @param query
   * @return The interpolation parameter on [0, 1] representing how far between this point and the
   *     upper point the query point lies.
   */
  double inverseInterpolate(T upper, T query);
}
