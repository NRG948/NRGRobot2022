// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Om WAS HERE TEST COMMIT!!

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandUtils;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.Interrupt;
import frc.robot.commands.ResetSubsystems;
import frc.robot.commands.SetModuleState;
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
  private JoystickButton xboxButtonY = new JoystickButton(driveController, 4); // y Button

  // Subsystems
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final RaspberryPiVision raspberryPiVision = new RaspberryPiVision();

  // Commands
  private final DriveWithController driveWithController = new DriveWithController(swerveDrive, driveController);
  private final Interrupt interrupt = new Interrupt(swerveDrive);
  private final DriveForward driveForward = new DriveForward(swerveDrive);
  private final SetModuleState setModuleState_0 = new SetModuleState(swerveDrive, driveController, 0);
  private final SetModuleState setModuleState_90 = new SetModuleState(swerveDrive, driveController, 90);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // swerveDrive.setDefaultCommand(driveWithController);
    // Configure the button bindings
    configureButtonBindings();

    // Init Shuffleboard
    swerveDrive.initShuffleboardTab();

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
    xboxButtonx.whenPressed(driveWithController);
    xboxButtonA.whenPressed(interrupt);
    xboxButtonB.whenPressed(setModuleState_0);
    xboxButtonY.whenPressed(setModuleState_90);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new ResetSubsystems(swerveDrive).andThen(
        CommandUtils.newFollowWaypointsCommand(swerveDrive,
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-1, -0.25)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-2, 0, Rotation2d.fromDegrees(-180)),
            true),
        new InstantCommand(() -> swerveDrive.stopMotors()));
  }

  public void initSubsystems() {
    raspberryPiVision.initPipeline();
  }
}
