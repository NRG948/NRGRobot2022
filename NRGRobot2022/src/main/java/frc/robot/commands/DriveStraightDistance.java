// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveDrive;

public class DriveStraightDistance extends DriveStraight {
  private final double distance;
  private Translation2d origin;
  
  /** Creates a new DriveStraightDistance. */
  public DriveStraightDistance(SwerveDrive sDrive, double speed, double heading, double distance) {
    super(sDrive, speed, heading, sDrive.getRotation2d().getDegrees());
    this.distance = distance;
  }

  public DriveStraightDistance(SwerveDrive sDrive, double speed, double heading, double distance, double orientation) {
    super(sDrive, speed, heading, orientation);
    this.distance = distance;
  }
  
  public DriveStraightDistance(SwerveDrive sDrive, double speed, Translation2d vector, double orientation){
    this(sDrive, speed, getHeading(vector), vector.getNorm(), orientation);
  }

  private static double getHeading(Translation2d vector){
    return Math.toDegrees(Math.atan2(vector.getY(), vector.getX()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    origin = swerveDrive.getPose2d().getTranslation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveDrive.getPose2d().getTranslation().getDistance(origin) >= distance;
  }
}
