package org.austincans.frc2158.subsystems.selfcheck;

import java.util.List;

public interface SelfChecking {
    List<SubsystemFault> checkForFaults();
}
