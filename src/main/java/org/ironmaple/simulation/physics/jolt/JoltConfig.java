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
//
// Updated on Saturday February 21, 2026 at 12:49:35 CST

package org.ironmaple.simulation.physics.jolt;

/**
 *
 *
 * <h1>Jolt Physics Configuration</h1>
 *
 * <p>Configuration settings for the Jolt physics engine.
 */
public class JoltConfig {
  /** Number of worker threads for Jolt's internal JobSystemThreadPool. */
  private int numWorkerThreads;

  /** Maximum number of bodies in the physics system. */
  private int maxBodies = 5_000;

  /** Maximum number of body pairs for collision detection. */
  private int maxBodyPairs = 65_536;

  /** Maximum number of contact points. */
  private int maxContacts = 20_480;

  /** Creates a default configuration using all available processors. */
  public JoltConfig() {
    this.numWorkerThreads = Runtime.getRuntime().availableProcessors();
  }

  /**
   * Creates a configuration with a specific number of worker threads.
   *
   * @param numWorkerThreads number of threads for physics calculations
   */
  public JoltConfig(int numWorkerThreads) {
    this.numWorkerThreads = numWorkerThreads;
  }

  public int getNumWorkerThreads() {
    return numWorkerThreads;
  }

  public JoltConfig setNumWorkerThreads(int numWorkerThreads) {
    this.numWorkerThreads = numWorkerThreads;
    return this;
  }

  public int getMaxBodies() {
    return maxBodies;
  }

  public JoltConfig setMaxBodies(int maxBodies) {
    this.maxBodies = maxBodies;
    return this;
  }

  public int getMaxBodyPairs() {
    return maxBodyPairs;
  }

  public JoltConfig setMaxBodyPairs(int maxBodyPairs) {
    this.maxBodyPairs = maxBodyPairs;
    return this;
  }

  public int getMaxContacts() {
    return maxContacts;
  }

  public JoltConfig setMaxContacts(int maxContacts) {
    this.maxContacts = maxContacts;
    return this;
  }

  /**
   * Creates a default configuration suitable for typical FRC simulation.
   *
   * @return default configuration
   */
  public static JoltConfig defaultConfig() {
    return new JoltConfig();
  }

  /**
   * Creates a lightweight configuration for resource-constrained environments.
   *
   * @return lightweight configuration with fewer threads and lower limits
   */
  public static JoltConfig lightweightConfig() {
    return new JoltConfig(2).setMaxBodies(1_000).setMaxBodyPairs(10_000).setMaxContacts(5_000);
  }
}
