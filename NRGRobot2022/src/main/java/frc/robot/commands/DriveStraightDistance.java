// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveStraightDistance extends DriveStraight {
  private final double distance;
  private Translation2d origin;
  /** Creates a new DriveStraightDistance. */
  public DriveStraightDistance(SwerveDrive sDrive, double speed, double heading, double distance) {
    super(sDrive, speed, heading);
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    origin = swerveDrive.getPose2d().getTranslation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveDrive.getPose2d().getTranslation().getDistance(origin) >= distance;
  }
}
