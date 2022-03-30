// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveStraightTo extends DriveStraightDistance {
  /** Creates a new DriveStraightTo. */
  public DriveStraightTo(SwerveDrive swerveDrive, double speed, Pose2d targetPose) {
    super(swerveDrive, speed, targetPose.getTranslation().minus(swerveDrive.getPose2d().getTranslation()), targetPose.getRotation().getDegrees());
  }

}
