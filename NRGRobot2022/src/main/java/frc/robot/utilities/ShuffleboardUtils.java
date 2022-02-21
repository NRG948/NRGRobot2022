// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/** Add your docs here. */
public class ShuffleboardUtils {
    /**
     * Adds a number slider widget to a Shuffleboard layout enabling a consumer to
     * receive updated values as the slider is changed.
     * 
     * @param layout       The Shuffleboard layout.
     * @param title        The title of the number slider widget.
     * @param defaultValue The default value of the slider widget.
     * @param consumer     The consumer of slider value changes.
     * 
     * @return A number slider widget.
     */
    public static SimpleWidget addNumberSlider(ShuffleboardLayout layout, String title, double defaultValue,
            DoubleConsumer consumer) {
        var sliderWidget = layout.add(title, defaultValue).withWidget(BuiltInWidgets.kNumberSlider);

        sliderWidget.getEntry().addListener(
                (event) -> consumer.accept(event.getEntry().getDouble(0.0)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        return sliderWidget;
    }
}
