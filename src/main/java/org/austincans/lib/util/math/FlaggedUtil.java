package org.austincans.lib.util.math;

import edu.wpi.first.units.Measure;

public class FlaggedUtil {
    public static boolean equals(boolean a, boolean b, boolean x) {
        return a && b && x || (!a && !b);
    }
    
    /**
     * @param a whether x applies
     * @param b whether y applies
     * @param x value
     * @param y value
     */
    public static boolean equals(boolean a, boolean b, double x, double y) {
        return equals(a, b, x == y);
    }
    
    public static boolean equals(boolean a, boolean b, Measure<?> x, Measure<?> y) {
        return equals(a, b, x.isEquivalent(y));
    }
}
