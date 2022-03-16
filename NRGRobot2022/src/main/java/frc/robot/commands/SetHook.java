// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.ClimberHooks.HookSelection;
import frc.robot.subsystems.ClimberHooks.State;

public class SetHook extends CommandBase {
  private ClimberHooks hooks;
  private State state;
  private HookSelection hookSelect;

  /** Creates a new SetHook. */
  public SetHook(ClimberHooks hooks, HookSelection hookSelect, State state) {
    this.hooks = hooks;
    this.hookSelect = hookSelect;
    this.state = state;
    
    addRequirements(hooks);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hooks.setState(state, hookSelect);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
