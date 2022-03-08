// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.DoubleValue;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.SwerveDrive;

public class DriveToCargo extends CommandBase {

  private SwerveDrive swerveDrive;
  private RaspberryPiVision pi;
  private boolean hasTarget;

  @RobotPreferencesValue
  public static DoubleValue cargoPickupSpeed = new DoubleValue("SwerveModule", "cargoPickupSpeed", 1.0);
  public static DoubleValue cargoPickupDistanceThreshold = new DoubleValue("SwerveModule", "cargoPickupDistanceThreshold", 1.0);
  /** Creates a new driveToBall. */
  public DriveToCargo(SwerveDrive swerveDrive, RaspberryPiVision pi) {
    this.swerveDrive = swerveDrive;
    this.pi = pi;
    hasTarget = true;
    addRequirements(this.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hasTarget = pi.hasTarget();
    if(hasTarget){
      double angle = pi.getAngleToTarget();
      swerveDrive.enableTurnToAngle(angle);
      double x = cargoPickupSpeed.getValue() * Math.cos(angle);
      double y = cargoPickupSpeed.getValue() * Math.sin(angle);
      swerveDrive.drive(x, y, 0, true, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !hasTarget || (pi.getDistanceToTarget() < cargoPickupDistanceThreshold.getValue());
  } 
}
