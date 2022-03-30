// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveDrive;

/** A command to drive in a straight line for the specified distance and heading. */
public class DriveStraightDistance extends DriveStraight {
  private final double distance;
  private Translation2d origin;

  /**
   * Constructs an instance of this class.
   * 
   * @param sDrive The swerve drivetrain.
   * @param speed The speed at which to drive.
   * @param heading The heading along which to drive.
   * @param distance The distance to drive.
   */
  public DriveStraightDistance(SwerveDrive sDrive, double speed, double heading, double distance) {
    this(sDrive, speed, heading, distance, sDrive.getRotation2d().getDegrees());
  }

  /**
   * Constructs an instance of this class.
   * 
   * @param sDrive The swerve drivetrain.
   * @param speed The speed at which to drive.
   * @param heading The heading along which to drive.
   * @param distance The distance to drive.
   * @param orientation The final orientation of the robot.
   */
  public DriveStraightDistance(SwerveDrive sDrive, double speed, double heading, double distance, double orientation) {
    super(sDrive, speed, heading, orientation);
    this.distance = distance;
  }
  

  /**
   * Constructs an instance of this class.
   * 
   * @param sDrive The swerve drivetrain.
   * @param speed The speed at which to drive.
   * @param vector The vector along which to drive.
   * @param orientation The final orientation of the robot.
   */
  public DriveStraightDistance(SwerveDrive sDrive, double speed, Translation2d vector, double orientation){
    this(sDrive, speed, getHeading(vector), vector.getNorm(), orientation);
  }

  /**
   * Returns the angle of the specified vector.
   * 
   * @param vector
   * @return The vector angle.
   */
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
