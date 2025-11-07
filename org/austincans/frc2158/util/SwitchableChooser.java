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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/** A string chooser for the dashboard where the options can be changed on-the-fly. */
public class SwitchableChooser extends LoggedNetworkInput {
    private static final String placeholder = "<NA>";

    private String[] options = new String[] {placeholder};
    private String active = placeholder;

    private final StringPublisher namePublisher;
    private final StringPublisher typePublisher;
    private final StringArrayPublisher optionsPublisher;
    private final StringPublisher defaultPublisher;
    private final StringPublisher activePublisher;
    private final StringPublisher selectedPublisher;
    private final LoggedNetworkString selectedInput;

    public SwitchableChooser(String name) {
        var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(name);
        namePublisher = table.getStringTopic(".name").publish();
        typePublisher = table.getStringTopic(".type").publish();
        optionsPublisher = table.getStringArrayTopic("options").publish();
        defaultPublisher = table.getStringTopic("default").publish();
        activePublisher = table.getStringTopic("active").publish();
        selectedPublisher = table.getStringTopic("selected").publish();
        selectedInput = new LoggedNetworkString("/SmartDashboard/" + name + "/selected");
        Logger.registerDashboardInput(this);

        namePublisher.set(name);
        typePublisher.set("String Chooser");
        optionsPublisher.set(this.options);
        defaultPublisher.set(this.options[0]);
        activePublisher.set(this.options[0]);
        selectedPublisher.set(this.options[0]);
    }

    /** Updates the set of available options. */
    public void setOptions(String[] options) {
        if (Arrays.equals(options, this.options)) {
            return;
        }
        this.options = options.length == 0 ? new String[] {placeholder} : options;
        optionsPublisher.set(this.options);
        periodic();
    }

    /** Returns the selected option. */
    public String get() {
        return active == placeholder ? null : active;
    }

    public void periodic() {
        String selected = selectedInput.get();
        active = null;
        for (String option : options) {
            if (!option.equals(placeholder) && option.equals(selected)) {
                active = option;
            }
        }
        if (active == null) {
            active = options[0];
            selectedPublisher.set(active);
        }
        defaultPublisher.set(active);
        activePublisher.set(active);
    }
}
