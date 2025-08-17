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
// Updated on Sunday August 17, 2025 at 08:59:14 CDT

package org.austincans.frc2158.util;

import java.util.Map;
import java.util.TreeMap;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined by making a guess
 * from points that are defined. This uses linear interpolation.
 *
 * @param <K> The type of the key (must implement InverseInterpolable)
 * @param <V> The type of the value (must implement Interpolable)
 * @author Austin Cans Robotics Team 2158
 */
public class InterpolatingTreeMap<
        K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>>
    extends TreeMap<K, V> {
  private static final long serialVersionUID = 8347275262778054124L;

  final int max_;

  public InterpolatingTreeMap(int maximumSize) {
    max_ = maximumSize;
  }

  public InterpolatingTreeMap() {
    this(0);
  }

  /**
   * Inserts a key value pair, and trims the tree if a max size is specified
   *
   * @param key Key for inserted data
   * @param value Value for inserted data
   * @return the value
   */
  @Override
  public V put(K key, V value) {
    if (max_ > 0 && max_ <= size()) {
      // "Prune" the tree if it is oversize
      K first = firstKey();
      remove(first);
    }

    super.put(key, value);

    return value;
  }

  @Override
  public void putAll(Map<? extends K, ? extends V> map) {
    System.out.println("Unimplemented Method");
  }

  /**
   * @param key Lookup for a value (does not have to exist)
   * @return V or null; V if it is Interpolable or exists, null if it is at a bound and cannot
   *     average
   */
  public V getInterpolated(K key) {
    V gotval = get(key);
    if (gotval == null) {
      // get surrounding keys for interpolation
      K topBound = ceilingKey(key);
      K bottomBound = floorKey(key);

      // if attempting interpolation at ends of tree, return the nearest data point
      if (topBound == null && bottomBound == null) {
        return null;
      } else if (topBound == null) {
        return get(bottomBound);
      } else if (bottomBound == null) {
        return get(topBound);
      }

      // get surrounding values for interpolation
      V topElem = get(topBound);
      V bottomElem = get(bottomBound);
      return bottomElem.interpolate(topElem, bottomBound.inverseInterpolate(topBound, key));
    } else {
      return gotval;
    }
  }
}
