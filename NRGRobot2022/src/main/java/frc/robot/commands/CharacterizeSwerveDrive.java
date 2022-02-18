// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class CharacterizeSwerveDrive extends CommandBase {

  private final SwerveDrive swerveDrive;

  private final NetworkTableEntry autoSpeedEntry =
          NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdAutoSpeed");
  private final NetworkTableEntry telemetryEntry =
          NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdTelemetry");

  private List<Double> telemetryData = new ArrayList<>();

  private double priorAutospeed = 0.0;

  /** Creates a new CharacterizeDrivetrainCommand. */
  public CharacterizeSwerveDrive(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double position = swerveDrive.getPose2d().getX();
    double velocity = swerveDrive.getChassisSpeeds().vxMetersPerSecond;

    double battery = RobotController.getBatteryVoltage();
    double motorVoltage = battery * Math.abs(priorAutospeed);

    double autospeed = autoSpeedEntry.getDouble(0.0);
    priorAutospeed = autospeed;

    swerveDrive.drive(autospeed, 0, 0, true, false);

    telemetryData.add(now);
    telemetryData.add(autospeed * RobotController.getInputVoltage());
    telemetryData.add(position);
    telemetryData.add(velocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    StringBuilder b = new StringBuilder();
    for (int i = 0; i < telemetryData.size(); ++i) {
        if (i != 0)
            b.append(", ");
        b.append(telemetryData.get(i));
    }
    telemetryEntry.setString(b.toString());

    swerveDrive.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
