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

import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.FileSystems;
import org.austincans.frc2158.config.Configuration;
import org.austincans.frc2158.config.ConfigurationLoader;

/*
 * This class is used to load the configuration from the config.yml file. The configuration
 * object is used to load the configuration from the config.yml file. The configuration object
 * is a static variable so that it can be accessed from anywhere in the code.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class ConfigUtil {

  private static final String CONFIG_FILE = "config.yml";

  // The configuration object is used to load the configuration from the config.yml file.
  // This is a static variable so that it can be accessed from anywhere in the code.
  private static Configuration config = getConfig();

  public ConfigUtil() {
    // Constructor is empty because we are using a static variable to hold the configuration.
    // This allows us to access the configuration from anywhere in the code without needing to
    // create an instance of this class.
  }

  // The configuration object is used to load the configuration from the config.yml file.
  // This is a static variable so that it can be accessed from anywhere in the code.
  public static int getConfigInt(String key, int defaultValue) {
    return (chkConfigValue(key) ? config.getInt(key) : defaultValue);
  }

  // The configuration object is used to load the configuration from the config.yml file.
  // This is a static variable so that it can be accessed from anywhere in the code.
  public static double getConfigDouble(String key, double defaultValue) {
    return (chkConfigValue(key) ? config.getDouble(key) : defaultValue);
  }

  // The configuration object is used to load the configuration from the config.yml file.
  // This is a static variable so that it can be accessed from anywhere in the code.
  public static boolean getConfigBoolean(String key, Boolean defaultValue) {
    return (chkConfigValue(key) ? config.getBoolean(key) : defaultValue);
  }

  // The configuration object is used to load the configuration from the config.yml file.
  // This is a static variable so that it can be accessed from anywhere in the code.
  public static String getConfigString(String key, String defaultValue) {
    return (chkConfigValue(key) ? config.getString(key) : defaultValue);
  }

  /**
   * Get the configuration object for the robot. This is used to load the configuration from the
   * config.yml file.
   *
   * @return The configuration object for the robot.
   */
  public Configuration getConfiguration() {
    Configuration config =
        ConfigurationLoader.fromFile(
                Filesystem.getDeployDirectory().getPath()
                    + FileSystems.getDefault().getSeparator()
                    + CONFIG_FILE)
            .load();
    return config;
  }

  public static boolean chkConfigValue(String configString) {
    Configuration config = getConfig();
    if (config.contains(configString)) {
      return true;
    } else {
      return false;
    }
  }

  // This function is used to get the configuration object for the robot.
  public static Configuration getConfig() {
    ConfigUtil objRobot = new ConfigUtil();
    return objRobot.getConfiguration();
  }
}
