// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.sysid.SysIdGeneralMechanismLogger;

public class CharacterizedArm extends CommandBase {
  private SysIdGeneralMechanismLogger logger = new SysIdGeneralMechanismLogger();
  private Arm arm;
  private double previousRadians;
  private double previousTime;

  /** Creates a new CharacterizedArm. */
  public CharacterizedArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.init();
    previousRadians = arm.getRadians();
    previousTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = 0.0;
    double currentTime = Timer.getFPGATimestamp();
    double currentRadians = arm.getRadians();
    double interval = currentTime - previousTime;

    if (interval != 0) {
      velocity = (currentRadians - previousRadians) / interval;
    }

    logger.log(currentRadians, velocity);
    arm.setMotorVoltage(logger.getMotorVoltage());

    previousTime = currentTime;
    previousRadians = currentRadians;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopMotor();
    logger.sendData();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled()
        || (logger.getMotorVoltage() > 0 ? arm.isAtStowedPosition() : arm.isAtRestingPosition());
  }
}
