// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.BooleanValue;

@RobotPreferencesLayout(groupName = "RaspberryPi", column = 5, row = 0, width = 1, height = 1)
public class RaspberryPiVision extends SubsystemBase {
  public static final String TARGET_PIPELINE_NAME_KEY = "Vision/Target/PipelineName";
  public static final String RED_CARGO_PIPELINE = "RedCargoPipeline";
  public static final String BLUE_CARGO_PIPELINE = "BlueCargoPipeline";
  @RobotPreferencesValue
  public static final BooleanValue enableTab = new BooleanValue("RaspberryPi", "enableTab", false);

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

  public String getPipeline() {
    return SmartDashboard.getString(TARGET_PIPELINE_NAME_KEY, BLUE_CARGO_PIPELINE);
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

  public boolean hasTarget() {
    return SmartDashboard.getBoolean("Vision/Target/HasTarget", false);
  }

  public double getDistanceToTarget() {
    return SmartDashboard.getNumber("Vision/Target/Distance", 0);
  }

  public double getAngleToTarget() {
    return SmartDashboard.getNumber("vision/Target/Angle", 0);
  }

  public void addShuffleboardTab() {
    if (!enableTab.getValue()) {
      return;
    }

    ShuffleboardTab piTab = Shuffleboard.getTab("Pi Vision");
    ShuffleboardLayout targetLayout = piTab.getLayout("Target Info", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 3);

    targetLayout.addBoolean("Pipeline", () -> getPipeline().compareTo(BLUE_CARGO_PIPELINE) == 0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#0000FF", "Color when false", "#FF0000"));
    targetLayout.addBoolean("Has Target", () -> hasTarget()).withWidget(BuiltInWidgets.kBooleanBox);
    targetLayout.addNumber("Distance", () -> getDistanceToTarget());
    targetLayout.addNumber("Angle", () -> getAngleToTarget());

    VideoSource processedVideo = new HttpCamera("Processed", "http://wpilibpi.local:1182/stream.mjpg");
    piTab.add("Processed Video", processedVideo)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);
  }
}
