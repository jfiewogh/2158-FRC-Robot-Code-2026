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
// Updated on Wednesday November 05, 2025 at 17:57:47 CST

package org.austincans.frc2158.config;

/**
 * An enum that specifies which paths should be returned when calling {@link
 * ConfigurationSection#getKeys(TraversalMode) getKeys} on a {@link Configuration} or {@link
 * ConfigurationSection} object.<br>
 * <br>
 *
 * <p>Assuming these are the paths relative to the {@link ConfigurationSection} on which {@link
 * ConfigurationSection#getKeys(TraversalMode) getKeys} is invoked on:
 *
 * <ul>
 *   <li>root.section.val1
 *   <li>root.section.val2
 *   <li>root.val1
 *   <li>root.val2
 *   <li>val1
 *   <li>val2
 * </ul>
 *
 * <br>
 *
 * <table>
 *  <caption style="display: none;">here you go</caption>
 *  <tr>
 *   <td style="vertical-align: top;">
 *    <strong>{@link #ROOT}</strong> would produce:
 *    <ul>
 *     <li>root</li>
 *     <li>val1</li>
 *     <li>val2</li>
 *    </ul>
 *   </td>
 *   <td style="vertical-align: top; padding-left: 3rem;">
 *    <strong>{@link #LEAVES}</strong> would produce:
 *    <ul>
 *     <li>root.section.val1</li>
 *     <li>root.section.val2</li>
 *     <li>root.val1</li>
 *     <li>root.val2</li>
 *     <li>val1</li>
 *     <li>val2</li>
 *    </ul>
 *   </td>
 *   <td style="vertical-align: top; padding-left: 3rem;">
 *    <strong>{@link #ALL}</strong> would produce:
 *    <ul>
 *     <li>root.section.val1</li>
 *     <li>root.section.val2</li>
 *     <li>root.section</li>
 *     <li>root.val1</li>
 *     <li>root.val2</li>
 *     <li>root</li>
 *     <li>val1</li>
 *     <li>val2</li>
 *    </ul>
 *   </td>
 *  </tr>
 * </table>
 *
 * <br>
 * <br>
 * <strong>Tip</strong>: to get branch paths only (i.e. those that are associated to {@link
 * ConfigurationSection}s rather than primitive or list values), one could do:
 *
 * <pre>{@code
 * Set<String> all = section.getKeys(ALL);
 * Set<String> leaves = section.getKeys(LEAVES);
 * Set<String> branches = all.minus(leaves);
 * }</pre>
 *
 * <p>which would return:
 *
 * <ul>
 *   <li>root.section
 *   <li>root
 * </ul>
 *
 * @since 2.0.0
 */
public enum TraversalMode {

  /**
   * Include root level paths only.
   *
   * @since 2.0.0
   */
  ROOT,

  /**
   * Include all leaf paths only.
   *
   * @since 2.0.0
   */
  LEAVES,

  /**
   * Include all paths (i.e.&nbsp;any branch or leaf path).
   *
   * <p>Note that root paths are branch paths too.
   *
   * @since 2.0.0
   */
  ALL
}
