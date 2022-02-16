// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.preferences;

import java.lang.module.Configuration;
import java.lang.reflect.Modifier;
import java.lang.reflect.Field;
import java.util.Set;
import java.util.stream.Stream;

import javax.naming.directory.ModificationItem;

import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ConfigurationBuilder;

import edu.wpi.first.wpilibj.Preferences;

public class RobotPreferences {

    public interface IValueVisitor{
        void visit(StringValue value);
        void visit(BooleanValue value);
        void visit(DoubleValue value);
    }
    
    public static abstract class Value{
        
        protected final String group;
        protected final String name;
        protected final String key;

        public Value(String group, String name){
            this.group = group;
            this.name = name;
            this.key = group + "/" + name;
        }

        public String getGroup(){
           return group; 
        }
        
        public String getName(){
            return name; 
        }
         
        public String getKey(){
            return key; 
        } 


        public boolean exists(){
            return Preferences.containsKey(key);
        }

        public abstract void accept(IValueVisitor visitor);
    }

    public static class StringValue extends Value {

        private final String defaultValue;

        public StringValue(String group, String name, String defaultValue){
            super(group, name);
            this.defaultValue = defaultValue;
        }
        
        public String getDefaultValue() {
            return defaultValue;
        }

        public String getValue(){
            return Preferences.getString(key, defaultValue);
        }

        public void setValue(String value){
            Preferences.setString(key, value);
        }

        @Override
        public void accept(IValueVisitor visitor) {
            visitor.visit(this);            
        }
    }

    public static class BooleanValue extends Value {

        private final boolean defaultValue;

        public BooleanValue(String group, String name, boolean defaultValue){
            super(group, name);
            this.defaultValue = defaultValue;
        }
        
        public boolean getDefaultValue() {
            return defaultValue;
        }

        public boolean getValue(){
            return Preferences.getBoolean(key, defaultValue);
        }

        public void setValue(Boolean value){
            Preferences.setBoolean(key, value);
        }

        @Override
        public void accept(IValueVisitor visitor) {
            visitor.visit(this);            
        }
    }

    public static class DoubleValue extends Value {

        private final double defaultValue;

        public DoubleValue(String group, String name, double defaultValue){
            super(group, name);
            this.defaultValue = defaultValue;
        }
        
        public double getDefaultValue() {
            return defaultValue;
        }

        public double getValue(){
            return Preferences.getDouble(key, defaultValue);
        }

        public void setValue(double value){
            Preferences.setDouble(key, value);
        }
    
        @Override
        public void accept(IValueVisitor visitor) {
            visitor.visit(this);            
        }
    }
    
    private static class DefaultValueWriter implements IValueVisitor{

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

    public static void init(){
        DefaultValueWriter writeDefaultValue = new DefaultValueWriter();
        getAllValues().filter(v -> !v.exists()).forEach(v -> v.accept(writeDefaultValue));

    }

    private static Stream<Field> getFields(){
        ConfigurationBuilder config = new ConfigurationBuilder()
            .forPackage("frc.robot")
            .setScanners(Scanners.FieldsAnnotated);
        
        Set<Field> fields = new Reflections(config)
            .get(Scanners.FieldsAnnotated
                .with(RobotPreferencesValue.class)
                .as(Field.class));
        return fields.stream().filter(f -> Modifier.isStatic(f.getModifiers()));
    }

    private static Value mapToValue(Field field){
        try{
            return (Value) field.get(null);
        } catch(IllegalArgumentException e){
            e.printStackTrace();
        } catch(IllegalAccessException e){
            e.printStackTrace();
        }

        return null;
    }  

    private static Stream<Value> getAllValues(){
        return getFields().map(RobotPreferences::mapToValue).filter(v -> v != null);
    }

}


