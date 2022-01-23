// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Om WAS HERE TEST COMMIT!!

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.Interrupt;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Operator interface (e.g. Joysticks)
  private final XboxController driveController = new XboxController(2);
  private JoystickButton xboxButtonA = new JoystickButton(driveController, 1); // A Button
  private JoystickButton xboxButtonB = new JoystickButton(driveController, 2); // B Button
  private JoystickButton xboxButtonx = new JoystickButton(driveController, 3); // x Button
  // Subsystems
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final RaspberryPiVision raspberryPiVision = new RaspberryPiVision();

  // Commands
  private final DriveWithController driveWithController = new DriveWithController(swerveDrive, driveController);
  private final Interrupt interrupt = new Interrupt(swerveDrive);
  private final DriveForward driveForward = new DriveForward(swerveDrive);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //swerveDrive.setDefaultCommand(driveWithController);
    // Configure the button bindings
    configureButtonBindings();
    xboxButtonx.whenPressed(driveWithController);
    ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Swerve Drive");

    ShuffleboardLayout absoluteEncoderValues = swerveDriveTab.getLayout("Absolute Encoder Values", BuiltInLayouts.kList);
    absoluteEncoderValues.addNumber("Front Left Turning Encoder", () ->  swerveDrive.getAbsoluteTurningEncoderPosition(0));
    absoluteEncoderValues.addNumber("Front Right Turning Encoder", () -> swerveDrive.getAbsoluteTurningEncoderPosition(1));
    absoluteEncoderValues.addNumber("Back Left Turning Encoder", () -> swerveDrive.getAbsoluteTurningEncoderPosition(2));
    absoluteEncoderValues.addNumber("Back Right Turning Encoder", () -> swerveDrive.getAbsoluteTurningEncoderPosition(3));

    ShuffleboardLayout relativeEncoderValues = swerveDriveTab.getLayout("Relative Encoder Values", BuiltInLayouts.kList);
    relativeEncoderValues.addNumber("Front Left Turning Encoder", () ->  swerveDrive.getRelativeTurningEncoderPosition(0));
    relativeEncoderValues.addNumber("Front Right Turning Encoder", () -> swerveDrive.getRelativeTurningEncoderPosition(1));
    relativeEncoderValues.addNumber("Back Left Turning Encoder", () -> swerveDrive.getRelativeTurningEncoderPosition(2));
    relativeEncoderValues.addNumber("Back Right Turning Encoder", () -> swerveDrive.getRelativeTurningEncoderPosition(3));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxButtonA.whenPressed(interrupt);
    xboxButtonB.whenPressed(driveForward);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
  
  public void initSubsystems() {
    raspberryPiVision.initPipeline();
  }
}
