// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;

   
@RobotPreferencesLayout(groupName = "ClimberModule", column = 2, row = 0, width = 2, height = 3, type = "Grid Layout")
public class ClimberRotator extends SubsystemBase {

  // TODO: Figure out column, row, width, and height for widget
  @RobotPreferencesValue
  public static DoubleValue climbingPower = new DoubleValue("ClimberModule", "Climbing Power", 0.5);
  @RobotPreferencesValue
  public static final DoubleValue verticalTolerance = new DoubleValue("ClimberModule", "Vertical Tolerance", 100);
  @RobotPreferencesValue
  public static final DoubleValue kP = new DoubleValue("ClimberModule", "kP", 0.0005);
  @RobotPreferencesValue
  public static final DoubleValue kI = new DoubleValue("ClimberModule", "kI", 0);
  @RobotPreferencesValue
  public static final DoubleValue kD = new DoubleValue("ClimberModule", "kD", 0);

  /** Creates a new ClimberRotator. */
  private final TalonFX climberMotor;

  public ClimberRotator() {
   climberMotor = new TalonFX(ClimberConstants.kClimberRotatorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Returns position of climber rotator. */
  public double getRotatorPosition() {
    return climberMotor.getSelectedSensorPosition();
  }
  /** Runs the climber motor with a power set via a preferences value. */
  public void rotateMotor() {
  //  climberMotor.set(climbingPower.getValue());
}

  public void rotateMotor(double power) {
    climberMotor.set(ControlMode.PercentOutput, power);
  }


/** Stops the climber motor. */
public void stopMotor() {
   climberMotor.set(ControlMode.PercentOutput, 0);
}

// Do not use this method. Similar code needs to be placed into Climber commands.
public void rotateTimedMotor(double d) {
    Timer time = new Timer();
    time.start();
    while (!time.hasElapsed(d)) {
      //  climberMotor.set(climbingPower.getValue());
    }
    time.stop();
}
}
