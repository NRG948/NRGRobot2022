// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class RunClaw extends CommandBase {

  private Claw claw;
  private double power;

  /** Creates a new RunClaw. */
  public RunClaw(Claw claw, double power) {
    this.claw = claw;
    this.power = power;
    addRequirements(this.claw);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running claw");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.claw.activateClaw(this.power);
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
