// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.sysid.SysIdDrivetrainLogger;

public class CharacterizeSwerveDrive extends CommandBase {

  private final SwerveDrive swerveDrive;

  private final SysIdDrivetrainLogger logger = new SysIdDrivetrainLogger();

  /** Creates a new CharacterizeDrivetrainCommand. */
  public CharacterizeSwerveDrive(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.init();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeed = swerveDrive.getChassisSpeeds();

    double position = swerveDrive.getPose2d().getX();
    double velocity = chassisSpeed.vxMetersPerSecond;
    double measuredAngle = swerveDrive.getRotation2d().getRadians();
    double angularRate = chassisSpeed.omegaRadiansPerSecond;

    double battery = RobotController.getBatteryVoltage();

    logger.log(position, velocity, measuredAngle, angularRate);

    double percentOutput = logger.getMotorVoltage() / battery;
    if (logger.isRotating()) {
      swerveDrive.drive(0, 0, percentOutput, true, false);
    } else {
      swerveDrive.drive(percentOutput, 0, 0, true, false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopMotors();
    logger.sendData();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled();
  }
}
