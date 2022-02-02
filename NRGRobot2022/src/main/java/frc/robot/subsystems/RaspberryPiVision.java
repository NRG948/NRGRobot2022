// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RaspberryPiVision extends SubsystemBase {
  public static final String TARGET_PIPELINE_NAME_KEY = "Vision/Target/PipelineName";
  public static final String RED_CARGO_PIPELINE = "RedCargoPipeline";
  public static final String BLUE_CARGO_PIPELINE = "BlueCargoPipeline";

  /** Creates a new RaspberryPiVision. */
  public RaspberryPiVision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPipeline(String pipelineName) {
    SmartDashboard.putString(TARGET_PIPELINE_NAME_KEY, pipelineName);
  }

  public void initPipeline() {
    DriverStation.Alliance alliance = DriverStation.getAlliance();
    switch (alliance) {
      default:
      case Blue:
        setPipeline(BLUE_CARGO_PIPELINE);
        break;
      case Red:
        setPipeline(RED_CARGO_PIPELINE);
        break;
    }
  }
}
