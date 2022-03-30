// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveDrive;

/** A command to drive in a straight line and rotates toward the target pose. */
public class DriveStraightTo extends DriveStraightDistance {
  /**
   * Constructs an instance of this command.
   * 
   * @param swerveDrive The swerve drivetrain.
   * @param speed The speed at which to drive.
   * @param targetPose The target pose.
   */
  public DriveStraightTo(SwerveDrive swerveDrive, double speed, Pose2d targetPose) {
    super(swerveDrive, speed, targetPose.getTranslation().minus(swerveDrive.getPose2d().getTranslation()), targetPose.getRotation().getDegrees());
  }
}
