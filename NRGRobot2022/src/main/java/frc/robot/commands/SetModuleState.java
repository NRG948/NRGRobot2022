// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SetModuleState extends CommandBase {
  private SwerveDrive swerveDrive;
  private XboxController driveController;
  private double angle = 0;

  /** Creates a new SetModuleState. */
  public SetModuleState(SwerveDrive sDrive, XboxController dController, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = sDrive;
    driveController = dController;
    this.angle = angle;
    addRequirements(swerveDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speed = driveController.getRightY()
    swerveDrive.setModuleState(3, -driveController.getLeftY(), angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
