// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.BooleanValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;

@RobotPreferencesLayout(groupName = "ClimberModule", column = 6, row = 0, width = 2, height = 3, type = "Grid Layout")
public class ClimberRotator extends SubsystemBase {
	
	@RobotPreferencesValue
	public static DoubleValue climbingPower = new DoubleValue("ClimberModule", "Climbing Power", 0.5);
	@RobotPreferencesValue
	public static final DoubleValue verticalTolerance = new DoubleValue("ClimberModule", "Vertical Tolerance", 1200);
	@RobotPreferencesValue
	public static final DoubleValue kP = new DoubleValue("ClimberModule", "kP", .00004);
	@RobotPreferencesValue
	public static final DoubleValue kI = new DoubleValue("ClimberModule", "kI", 0); // Not used
	@RobotPreferencesValue
	public static final DoubleValue kD = new DoubleValue("ClimberModule", "kD", 0); // Not used
	@RobotPreferencesValue
	public static final BooleanValue enableTab = new BooleanValue("ClimberModule", "enableTab", false);

	/** Creates a new ClimberRotator. */
	private final TalonFX climberMotor;

	public ClimberRotator() {
		climberMotor = new TalonFX(ClimberConstants.kClimberRotatorMotor);
		climberMotor.setSelectedSensorPosition(0);
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
		climberMotor.set(ControlMode.PercentOutput, climbingPower.getValue());
	}

	public void rotateMotor(double power) {
		climberMotor.set(ControlMode.PercentOutput, power);
	}


	/** Stops the climber motor. */
	public void stopMotor() {
		climberMotor.set(ControlMode.PercentOutput, 0);
	}

	// Do not use this method. Similar code needs to be placed into Climber
	// commands.
	public void rotateTimedMotor(double d) {
		Timer time = new Timer();
		time.start();
		while (!time.hasElapsed(d)) {
			// climberMotor.set(climbingPower.getValue());
		}
		time.stop();
	}

	public void addShuffleboardLayout(ShuffleboardTab climberTab) {
		ShuffleboardLayout rotatorLayout = climberTab.getLayout("Rotator", BuiltInLayouts.kGrid)
				.withPosition(0, 0)
				.withSize(2, 2);
		rotatorLayout.addNumber("Encoder", () -> climberMotor.getSelectedSensorPosition());
	}
}
