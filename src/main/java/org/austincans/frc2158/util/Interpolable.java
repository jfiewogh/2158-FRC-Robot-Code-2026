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
// Updated on Thursday November 13, 2025 at 17:43:29 CST

package org.austincans.frc2158.util;

/**
 * Interpolable is an interface used by an Interpolating Tree as the Value type. Given two end
 * points and an interpolation parameter on [0, 1], it calculates a new Interpolable representing
 * the interpolated value.
 *
 * @param <T> The Type of Interpolable
 * @see InterpolatingTreeMap
 * @author Austin Cans Robotics Team 2158
 */
public interface Interpolable<T> {
  /**
   * Interpolates between this value and an other value according to a given parameter. If x is 0,
   * the method should return this value. If x is 1, the method should return the other value. If 0
   * < x < 1, the return value should be interpolated proportionally between the two.
   *
   * @param other The value of the upper bound
   * @param x The requested value. Should be between 0 and 1.
   * @return Interpolable<T> The estimated average between the surrounding data
   */
  T interpolate(T other, double x);
}
