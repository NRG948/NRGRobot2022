// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveStraight extends CommandBase {
  private final SwerveDrive swerveDrive;
  private final double xSpeed;
  private final double ySpeed;

  /**
   * Constructs an instance of this class.
   * 
   * @param sDrive  An instance of the swerve drive subsystem.
   * @param speed   Speed to drive.
   * @param heading The direction in radians to drive.
   */
  public DriveStraight(SwerveDrive sDrive, double speed, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = sDrive;
    addRequirements(swerveDrive);
    xSpeed = speed * Math.cos(heading);
    ySpeed = speed * Math.sin(heading);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(xSpeed, ySpeed, 0, true, false);
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
