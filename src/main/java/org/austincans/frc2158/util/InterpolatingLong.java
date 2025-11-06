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

/**
 * A Long that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 * @author Austin Cans Robotics Team 2158
 */
public class InterpolatingLong
    implements Interpolable<InterpolatingLong>,
        InverseInterpolable<InterpolatingLong>,
        Comparable<InterpolatingLong> {
  public Long value = 0L;

  public InterpolatingLong(Long val) {
    value = val;
  }

  @Override
  public InterpolatingLong interpolate(InterpolatingLong other, double x) {
    Long dydx = other.value - value;
    Double searchY = dydx * x + value;
    return new InterpolatingLong(searchY.longValue());
  }

  @Override
  public double inverseInterpolate(InterpolatingLong upper, InterpolatingLong query) {
    long upper_to_lower = upper.value - value;
    if (upper_to_lower <= 0) {
      return 0;
    }
    long query_to_lower = query.value - value;
    if (query_to_lower <= 0) {
      return 0;
    }
    return query_to_lower / (double) upper_to_lower;
  }

  @Override
  public int compareTo(InterpolatingLong other) {
    if (other.value < value) {
      return 1;
    } else if (other.value > value) {
      return -1;
    } else {
      return 0;
    }
  }
}
