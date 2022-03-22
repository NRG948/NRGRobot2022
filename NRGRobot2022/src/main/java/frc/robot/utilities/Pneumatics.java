// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.preferences.RobotPreferences;

/** Add your docs here. */
public class Pneumatics {

    public static PneumaticsModuleType getModuleType() {
        return RobotPreferences.practiceBot.getValue() ? PneumaticsModuleType.CTREPCM : PneumaticsModuleType.REVPH;
    }
}
