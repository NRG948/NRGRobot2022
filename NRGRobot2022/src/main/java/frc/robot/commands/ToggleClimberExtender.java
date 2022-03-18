// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberExtender;

/**
 * command which toggles climberExtender state.
 */
public class ToggleClimberExtender extends CommandBase {

  private ClimberExtender climberExtender;

  /** Creates a new ToggleClimberExtender. */
  public ToggleClimberExtender(ClimberExtender climberExtender) {
    this.climberExtender = climberExtender;
    addRequirements(this.climberExtender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Changes from 3/17
    System.out.println("Toggle Climber Extender");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberExtender.toggleState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
