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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

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
