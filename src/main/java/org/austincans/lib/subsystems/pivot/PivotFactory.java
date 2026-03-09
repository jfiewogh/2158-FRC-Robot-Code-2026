package org.austincans.lib.subsystems.pivot;

import org.austincans.lib.configs.PivotConfig;

public class PivotFactory {
    public Pivot createArmTalonFXS(PivotConfig pivotConfig) {
        return new Pivot(new PivotIOTalonFXSCANcoder(pivotConfig));
    } 

    public Pivot createArmSparkFlex(PivotConfig pivotConfig) {
        return new Pivot(new PivotIOSparkFlexCANcoder(pivotConfig));
    } 
}
