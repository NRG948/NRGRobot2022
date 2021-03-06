// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/** A command to drive in a straight line. */
public class DriveStraight extends CommandBase {
  protected final SwerveDrive swerveDrive;
  private double speed;
  protected double heading;
  protected double xSpeed;
  protected double ySpeed;
  private final double orientation;

  /**
   * Constructs an instance of this class.
   * 
   * @param swerveDrive An instance of the swerve drive subsystem.
   * @param speed       Speed to drive.
   * @param heading     The direction in degrees to drive.
   * @param orientation The desired orientation of the robot
   */
  public DriveStraight(SwerveDrive swerveDrive, double speed, double heading, double orientation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrive = swerveDrive;
    this.speed = speed;
    this.orientation = orientation;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double headingRadians = Math.toRadians(heading);
    xSpeed = speed * Math.cos(headingRadians);
    ySpeed = speed * Math.sin(headingRadians);
    swerveDrive.enableTurnToAngle(orientation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(xSpeed, ySpeed, 0, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopMotors();
    swerveDrive.disableTurnToAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
