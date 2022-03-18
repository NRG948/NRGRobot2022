// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberRotator;

public class KeepClimberRotatorVertical extends CommandBase {
  private ClimberRotator climberRotator;

  /** Creates a new KeepClimberRotatorVertical. */
  public KeepClimberRotatorVertical(ClimberRotator climberRotator) {
    this.climberRotator = climberRotator;
    addRequirements(climberRotator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Changes from 3/17
    System.out.println("Keep Climber Rotator Vertical");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tolerance = ClimberRotator.verticalTolerance.getValue();
    double position = climberRotator.getRotatorPosition();
    double power = 0;

    if(Math.abs(position) > tolerance) {
      power = -ClimberRotator.kP.getValue() * position; // do  we need to negate this?
    }
    System.out.println("Rotator Power: " + power);
    climberRotator.rotateMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberRotator.stopMotor();
  }

  // this command runs until interrupted.
  @Override
  public boolean isFinished() {
    return false;
  }
}
