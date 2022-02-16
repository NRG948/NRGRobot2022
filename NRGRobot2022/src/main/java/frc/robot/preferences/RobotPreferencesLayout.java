// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.preferences;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Add your docs here. */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface RobotPreferencesLayout {
    String groupName();
    int column();
    int row();
    int width();
    int height();
}
