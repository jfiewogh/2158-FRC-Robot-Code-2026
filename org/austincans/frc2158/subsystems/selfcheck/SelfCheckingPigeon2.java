package org.austincans.frc2158.subsystems.selfcheck;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPigeon2 implements SelfChecking {
    private final String label;
    private final Pigeon2 pigeon;

    public SelfCheckingPigeon2(String label, Pigeon2 pigeon) {
        this.label = label;
        this.pigeon = pigeon;
    }

    @Override
    public List<SubsystemFault> checkForFaults() {
        List<SubsystemFault> faults = new ArrayList<>();

        if (this.pigeon.getFault_Hardware(true).getValue()) {
            faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
        }
        if (this.pigeon.getFault_BootDuringEnable(true).getValue()) {
            faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
        }
        if (this.pigeon.getFault_BootIntoMotion(true).getValue()) {
            faults.add(new SubsystemFault(String.format("[%s]: Device booted while in motion", label)));
        }
        if (this.pigeon.getFault_BootupGyroscope(true).getValue()) {
            faults.add(new SubsystemFault(String.format("[%s]: Gyro fault detected", label)));
        }
        if (this.pigeon.getFault_BootupAccelerometer().getValue()) {
            faults.add(new SubsystemFault(String.format("[%s]: Accelerometer fault detected", label)));
        }

        StatusSignal<Integer> faultsError = pigeon.getFaultField(
                true); // fills faults with the current Pigeon 2 faults; returns the last error generated
        if (faultsError.getValue() != 0) {
            faults.add(new SubsystemFault(String.format("[%s]: Error Code (%s)", label, faultsError.getValue())));
        }

        return faults;
    }
}
