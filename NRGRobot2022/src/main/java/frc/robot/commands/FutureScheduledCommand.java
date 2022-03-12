// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FutureScheduledCommand extends CommandBase {
  private final FutureSupplier<CommandBase> commandSupplier;
  private CommandBase command;
  private Future<CommandBase> futureCommand;

  /** Creates a new FutureScheduledCommand. */
  public FutureScheduledCommand(FutureSupplier<CommandBase> commandSupplier) {
    this.commandSupplier = commandSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    futureCommand = commandSupplier.getFuture(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (futureCommand.isDone() && command == null) {
      try {
        command = futureCommand.get();
        command.schedule();
      } catch (InterruptedException | ExecutionException e) {
        e.printStackTrace();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted && command != null) {
      command.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (command != null) {
      return !command.isScheduled();
    }
    return false;
  }
}
