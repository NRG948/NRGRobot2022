// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.preferences;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Set;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ConfigurationBuilder;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A class to manage robot preferences. */
public class RobotPreferences {

    /** An interface to support the Visitor pattern on preferences values. */
    public interface IValueVisitor {

        /** Called to apply the visitors effect on a StringValue. */
        void visit(StringValue value);

        /** Called to apply the visitors effect on a BooleanValue. */
        void visit(BooleanValue value);

        /** Called to apply the visitors effect on a DoubleValue. */
        void visit(DoubleValue value);
    }

    /**
     * An abstract base class represented a keyed valued in the preferences store.
     */
    public static abstract class Value {

        protected final String group;
        protected final String name;
        protected final String key;

        /**
         * Constructs an instance of this class.
         * 
         * @param group The preference value's group name. This is usually the subsystem
         *              or class that uses the value.
         * @param name  The preferences value name.
         */
        public Value(String group, String name) {
            this.group = group;
            this.name = name;
            this.key = group + "/" + name;
        }

        /** Returns the this value's group name. */
        public String getGroup() {
            return group;
        }

        /** Returns this value's name. */
        public String getName() {
            return name;
        }

        /**
         * Returns this value key used to access it in the preferences store. The value
         * is generated from a concatenation of the group name and value name separated
         * by a forward slash ("/") character.
         */
        public String getKey() {
            return key;
        }

        /** Returns whether the value exists in the preferences store. */
        public boolean exists() {
            return Preferences.containsKey(key);
        }

        /** Accepts a visitor that operates on this value. */
        public abstract void accept(IValueVisitor visitor);
    }

    /** A class representing a string value in the preferences store. */
    public static class StringValue extends Value {

        private final String defaultValue;

        /**
         * Constructs an instance of this class.
         * 
         * @param group        The preference value's group name. This is usually the
         *                     subsystem
         *                     or class that uses the value.
         * @param name         The preferences value name.
         * @param defaultValue The value supplied when the preference does not exist in
         *                     the preferences store.
         */
        public StringValue(String group, String name, String defaultValue) {
            super(group, name);
            this.defaultValue = defaultValue;
        }

        /** Returns the default value. */
        public String getDefaultValue() {
            return defaultValue;
        }

        /** Returns the current value. */
        public String getValue() {
            return Preferences.getString(key, defaultValue);
        }

        /**
         * Sets the current value.
         * 
         * @param value The value to set.
         */
        public void setValue(String value) {
            Preferences.setString(key, value);
        }

        @Override
        public void accept(IValueVisitor visitor) {
            visitor.visit(this);
        }
    }

    /** A class representing a Boolean value in the preferences store. */
    public static class BooleanValue extends Value {

        private final boolean defaultValue;

        /**
         * Constructs an instance of this class.
         * 
         * @param group        The preference value's group name. This is usually the
         *                     subsystem
         *                     or class that uses the value.
         * @param name         The preferences value name.
         * @param defaultValue The value supplied when the preference does not exist in
         *                     the preferences store.
         */
        public BooleanValue(String group, String name, boolean defaultValue) {
            super(group, name);
            this.defaultValue = defaultValue;
        }

        /** Returns the default value. */
        public boolean getDefaultValue() {
            return defaultValue;
        }

        /** Returns the current value. */
        public boolean getValue() {
            return Preferences.getBoolean(key, defaultValue);
        }

        /**
         * Sets the current value.
         * 
         * @param value The value to set.
         */
        public void setValue(Boolean value) {
            Preferences.setBoolean(key, value);
        }

        @Override
        public void accept(IValueVisitor visitor) {
            visitor.visit(this);
        }
    }

    /** A class representing a floating-point value in the preferences store. */
    public static class DoubleValue extends Value {

        private final double defaultValue;

        /**
         * Constructs an instance of this class.
         * 
         * @param group        The preference value's group name. This is usually the
         *                     subsystem
         *                     or class that uses the value.
         * @param name         The preferences value name.
         * @param defaultValue The value supplied when the preference does not exist in
         *                     the preferences store.
         */
        public DoubleValue(String group, String name, double defaultValue) {
            super(group, name);
            this.defaultValue = defaultValue;
        }

        /** Returns the default value. */
        public double getDefaultValue() {
            return defaultValue;
        }

        /** Returns the current value. */
        public double getValue() {
            return Preferences.getDouble(key, defaultValue);
        }

        /**
         * Sets the current value.
         * 
         * @param value The value to set.
         */
        public void setValue(double value) {
            Preferences.setDouble(key, value);
        }

        @Override
        public void accept(IValueVisitor visitor) {
            visitor.visit(this);
        }
    }

    /** A Visitor that writes the default preferences value to the store. */
    private static class DefaultValueWriter implements IValueVisitor {

        @Override
        public void visit(StringValue value) {
            value.setValue(value.getDefaultValue());
        }

        @Override
        public void visit(BooleanValue value) {
            value.setValue(value.getDefaultValue());
        }

        @Override
        public void visit(DoubleValue value) {
            value.setValue(value.getDefaultValue());
        }
    }
    
    /** A Visitor to print non default Values to the console. */
    private static class NonDefaultValuePrinter implements IValueVisitor {
        private static void printMessage(String group, String name, String value) {
            System.out.println("NON-DEFAULT VALUE: " + group + "/" + name + " = " + value);
        }

        @Override
        public void visit(StringValue value) {
            if (!value.getValue().equals(value.getDefaultValue())) {
                printMessage(value.getGroup(), value.getName(), value.getValue());
            }
        }

        @Override
        public void visit(BooleanValue value) {
            if (value.getValue() != value.getDefaultValue()) {
                printMessage(value.getGroup(), value.getName(), Boolean.toString(value.getValue()));
            }
        }

        @Override
        public void visit(DoubleValue value) {
            if (value.getValue() != value.getDefaultValue()) {
                printMessage(value.getGroup(), value.getName(), Double.toString(value.getValue()));
            }
        }
    }

    /** A Visitor that adds widgets to the Shuffleboard preferences layout. */
    private static class ShuffleboardWidgetBuilder implements IValueVisitor {

        private ShuffleboardLayout layout;

        public ShuffleboardWidgetBuilder(ShuffleboardLayout layout) {
            this.layout = layout;
        }

        @Override
        public void visit(StringValue value) {
            layout.add(value.getName(), value.getValue())
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry()
                    .addListener(
                            (event) -> value.setValue(event.getEntry().getString(value.getDefaultValue())),
                            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        }

        @Override
        public void visit(BooleanValue value) {
            layout.add(value.getName(), value.getValue())
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .getEntry()
                    .addListener(
                            (event) -> value.setValue(event.getEntry().getBoolean(value.getDefaultValue())),
                            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }

        @Override
        public void visit(DoubleValue value) {
            layout.add(value.getName(), value.getValue())
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry()
                    .addListener(
                            (event) -> value.setValue(event.getEntry().getDouble(value.getDefaultValue())),
                            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }

    }

    /** Initializes the robot preferences. */
    public static void init() {
        DefaultValueWriter writeDefaultValue = new DefaultValueWriter();

        getAllValues().filter(v -> !v.exists()).forEach(v -> v.accept(writeDefaultValue));
        
        NonDefaultValuePrinter printer = new NonDefaultValuePrinter();

        getAllValues().forEach((v) -> v.accept(printer));
    }

    /**
     * Adds a tab to the Shuffleboard allowing the robot operator to adjust values.
     */
    public static void addShuffleBoardTab() {
        ShuffleboardTab prefsTab = Shuffleboard.getTab("Preferences");

        ConfigurationBuilder config = new ConfigurationBuilder()
                .forPackage("frc.robot")
                .setScanners(Scanners.TypesAnnotated);
        Set<Class<?>> classes = new Reflections(config)
                .get(Scanners.TypesAnnotated
                        .with(RobotPreferencesLayout.class)
                        .asClass());

        classes.stream().map(c -> c.getAnnotation(RobotPreferencesLayout.class)).forEach(layout -> {
            prefsTab.getLayout(layout.groupName(), BuiltInLayouts.kList)
                    .withPosition(layout.column(), layout.row())
                    .withSize(layout.width(), layout.height());
        });

        getAllValues().collect(Collectors.groupingBy(Value::getGroup)).forEach((group, values) -> {
            ShuffleboardLayout layout = prefsTab.getLayout(group);
            ShuffleboardWidgetBuilder builder = new ShuffleboardWidgetBuilder(layout);
            values.stream().forEach((value) -> value.accept(builder));
        });
    }

    /** Returns a stream of fields containing preferences values. */
    private static Stream<Field> getFields() {
        ConfigurationBuilder config = new ConfigurationBuilder()
                .forPackage("frc.robot")
                .setScanners(Scanners.FieldsAnnotated);
        Set<Field> fields = new Reflections(config)
                .get(Scanners.FieldsAnnotated
                        .with(RobotPreferencesValue.class)
                        .as(Field.class));

        return fields.stream().filter(f -> Modifier.isStatic(f.getModifiers()));
    }

    /** Maps a preferences field to its value instance. */
    private static Value mapToValue(Field field) {
        try {
            return (Value) field.get(null);
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        return null;
    }

    /** Returns all preferences values. */
    private static Stream<Value> getAllValues() {
        return getFields().map(RobotPreferences::mapToValue).filter(v -> v != null);
    }

}
