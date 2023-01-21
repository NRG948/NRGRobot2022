// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberRotator;

/**
 * A command that ramps the rotator motor power from a minPower to a maxPower over a period of seconds
 */

public class RampRotatorMotor extends CommandBase {
  private ClimberRotator rotator;
  private double minPower;
  private double maxPower;
  private double rampTimeSeconds;
  private Timer timer;
  private double slope;

  /** Creates a new RampRotatorMotor. */
  public RampRotatorMotor(ClimberRotator rotator, double minPower, double maxPower, double rampTimeSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rotator = rotator;
    this.minPower = minPower;
    this.maxPower = maxPower;
    this.rampTimeSeconds = rampTimeSeconds;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    slope = (maxPower-minPower)/rampTimeSeconds;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotator.rotateMotor(Math.min(minPower + (slope * timer.get()), maxPower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
