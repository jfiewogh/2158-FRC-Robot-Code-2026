package org.austincans.lib.io.motors.configs;

import org.austincans.lib.util.math.FlaggedUtil;

import edu.wpi.first.units.measure.Angle;

/** Soft limits should be in mechanism values. */
public class SoftLimitConfig extends Object {
    public boolean forwardSoftLimitEnabled;
    public Angle forwardSoftLimit;
    public boolean reverseSoftLimitEnabled;
    public Angle reverseSoftLimit;

    public boolean equals(SoftLimitConfig other) {
        return FlaggedUtil.equals(
                forwardSoftLimitEnabled, other.forwardSoftLimitEnabled, 
                forwardSoftLimit, other.forwardSoftLimit)
            && FlaggedUtil.equals(
                reverseSoftLimitEnabled, other.reverseSoftLimitEnabled, 
                reverseSoftLimit, other.reverseSoftLimit);
    }
}