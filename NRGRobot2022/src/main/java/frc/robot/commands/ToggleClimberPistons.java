// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ToggleClimberPistons extends CommandBase {

  private Climber climber;
  private DoubleSolenoid piston;
  /** Creates a new ToggleClimberPistons. */
  public ToggleClimberPistons(Climber climber, int pistonNumber) {
    this.climber = climber;
    this.piston = pistonNumber == 1 ? climber.getP1() : climber.getP2();
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.toggleState(piston);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
