package org.austincans.lib.subsystems.pivot;

public class PivotFactory {
    public Pivot createArmTalonFXS(PivotConfig armConfig) {
        return new Pivot(new PivotIOTalonFXSCANcoder(armConfig));
    } 

    public Pivot createArmSparkFlex(PivotConfig armConfig) {
        return new Pivot(new PivotIOSparkFlexCANcoder(armConfig));
    } 
}
