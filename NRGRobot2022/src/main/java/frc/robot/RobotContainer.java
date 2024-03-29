// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Om WAS HERE TEST COMMIT!!

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoClaw;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.ManualClaw;
import frc.robot.commands.ManualClimber;
import frc.robot.commands.RotateArmToResting;
import frc.robot.commands.RotateArmToScoring;
import frc.robot.commands.RotateArmToScoring2;
import frc.robot.commands.RotateArmToStowed;
import frc.robot.commands.ToggleClimberExtender;
import frc.robot.commands.TurnToAngle;
import frc.robot.preferences.RobotPreferences;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClimberExtender;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.ClimberRotator;
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
  private JoystickButton driverButtonA = new JoystickButton(driveController, 1); // A Button
  private JoystickButton driverButtonB = new JoystickButton(driveController, 2); // B Button
  private JoystickButton driverButtonX = new JoystickButton(driveController, 3); // X Button
  private JoystickButton driverButtonY = new JoystickButton(driveController, 4); // Y Button
  private JoystickButton driverLeftBumper = new JoystickButton(driveController, 5);
  private JoystickButton driverRightBumper = new JoystickButton(driveController, 6);
  // Left Middle Button
  private JoystickButton driverStartButton = new JoystickButton(driveController, 7);
  // Right Middle Button
  private JoystickButton driverMenuButton = new JoystickButton(driveController, 8);

  private POVButton driverDpadUp = new POVButton(driveController, 0);
  private POVButton driverDpadRight = new POVButton(driveController, 90);
  private POVButton driverDpadDown = new POVButton(driveController, 180);
  private POVButton driverDpadLeft = new POVButton(driveController, 270);

  // Manipulate interface
  private final XboxController manipulatorController = new XboxController(3);
  private JoystickButton manipulatorButtonA = new JoystickButton(manipulatorController, 1); // A Button
  private JoystickButton manipulatorButtonB = new JoystickButton(manipulatorController, 2); // B Button
  private JoystickButton manipulatorButtonX = new JoystickButton(manipulatorController, 3); // X Button
  private JoystickButton manipulatorButtonY = new JoystickButton(manipulatorController, 4); // Y Button
  private JoystickButton manipulatorLeftBumper = new JoystickButton(manipulatorController, 5);
  private JoystickButton manipulatorRightBumper = new JoystickButton(manipulatorController, 6);
  private JoystickButton manipulatorStartButton = new JoystickButton(manipulatorController, 7);
  private JoystickButton manipulatorMenuButton = new JoystickButton(manipulatorController, 8);
  private POVButton manipulatorDpadUp = new POVButton(manipulatorController, 0);
  private POVButton manipulatorDpadRight = new POVButton(manipulatorController, 90);
  private POVButton manipulatorDpadDown = new POVButton(manipulatorController, 180);
  private POVButton manipulatorDpadLeft = new POVButton(manipulatorController, 270);

  // Subsystems
  public static final SwerveDrive swerveDrive = new SwerveDrive();
  public static final RaspberryPiVision raspberryPiVision = new RaspberryPiVision();
  public static final Claw claw = new Claw(1); // Port 1
  public static final Arm arm = new Arm();
  public static final ClimberExtender climberExtender = new ClimberExtender();
  public static final ClimberHooks climberHooks = new ClimberHooks();
  public static final ClimberRotator climberRotator = new ClimberRotator();
  public static final Subsystem[] allSubsystems = new Subsystem[] {swerveDrive, raspberryPiVision, claw, arm, climberExtender, climberHooks/*, climberRotator*/};

  // Commands
  private final DriveWithController driveWithController = new DriveWithController(swerveDrive, driveController);
  private final ManualClaw manualClaw = new ManualClaw(claw, manipulatorController);
  private final RotateArmToResting armToResting = new RotateArmToResting(arm);
  private final RotateArmToScoring armToScoring = new RotateArmToScoring(arm);
  // private final ManualClimber manualClimber = new ManualClimber(climberRotator, manipulatorController);

  // // Raise the climber, drive to the MID RUNG & grab it when detected
  // private static final SequentialCommandGroup climbSequencePart1 =
  //     new InstantCommand(() -> swerveDrive.resetHeading())
  //         .andThen(new InstantCommand(() -> climberExtender.setState(ClimberExtender.State.UP))
  //           .alongWith(new InstantCommand(() -> {arm.setGoal(Math.toRadians(75)); arm.enable();})))
  //         .andThen(new WaitCommand(3.0)) // wait for extender to go up
  //         .andThen(new DriveStraight(swerveDrive, .15, 180) // Backup slowly into MID RUNG
  //             .until(() -> climberHooks.isBarDetected(HookSelection.HOOK_1))
  //         .andThen(new WaitCommand(.1)));

  // Back up until arm passes vertical point, rotate the climber, grab HIGH RUNG
  // private static final SequentialCommandGroup climbSequencePart2 =
  //         new DriveStraight(swerveDrive, .1, 0) // Slowly moving forward
  //             .until(() -> climberRotator.getRotatorPosition() > 800) // TBD
  //         .andThen(new PerpetualCommand(new InstantCommand(() -> climberRotator.rotateMotor(), climberRotator))
  //             .until(() -> climberHooks.isBarDetected(HookSelection.HOOK_2)))
  //         .andThen(new PerpetualCommand(new InstantCommand(() -> climberRotator.rotateMotor(), climberRotator))
  //             .until(() -> !climberHooks.isBarDetected(HookSelection.HOOK_2)))
  //         .andThen(new InstantCommand(() -> climberRotator.stopMotor()));

  // // Release MID RUNG, wait, climb to TRAVERSAL RUNG and grab it
  // private static final SequentialCommandGroup climbSequencePart3 =
  //         new InstantCommand(() -> climberRotator.backDriveMotor())
  //         .andThen(new WaitCommand(0.3))
  //         .andThen(new InstantCommand(() -> climberRotator.stopMotor())
  //         .andThen(new WaitCommand(2)) // wait for robot to swing
  //         .andThen(new PerpetualCommand(new InstantCommand(() -> climberRotator.rotateMotor(), climberRotator))
  //             .until(() -> climberHooks.isBarDetected(HookSelection.HOOK_1)))
  //         .andThen(new PerpetualCommand(new InstantCommand(() -> climberRotator.rotateMotor(), climberRotator))
  //             .until(() -> !climberHooks.isBarDetected(HookSelection.HOOK_1)))
  //         .andThen(new InstantCommand(() -> climberRotator.stopMotor())))
  //         .andThen(new WaitCommand(1.0));

  // // Release the High rung, wait 3/10s of a second, stop motor.        
  // private static final SequentialCommandGroup climbSequencePart4 =
  //     new InstantCommand(() -> climberRotator.backDriveMotor())
  //         .andThen(new WaitCommand(0.3))
  //         .andThen(new InstantCommand(() -> climberRotator.stopMotor()));
          
  // private static final SequentialCommandGroup climbUnlatch = 
  //     new InstantCommand(() -> climberRotator.backDriveMotor())
  //         .andThen(new WaitCommand(0.3))
  //         .andThen(() -> climberRotator.stopMotor());

  //         private static final SequentialCommandGroup climbUnlatchTwo = 
  //     new InstantCommand(() -> climberRotator.backDriveMotor())
  //         .andThen(new WaitCommand(0.6))
  //         .andThen(() -> climberRotator.stopMotor());

  // // Fully autonomous 3-stage traversal climb
  // private static final SequentialCommandGroup climbSequenceFull = null;
  //     // new SequentialCommandGroup(climbSequencePart1, climbSequencePart2, climbSequencePart3);

  // private static final Command abortClimb = new InstantCommand(() -> {}, allSubsystems)
  //   .andThen(new PerpetualCommand(new InstantCommand(() -> climberRotator.rotateMotor(-0.15), climberRotator)))
  //     .until(() -> climberRotator.getRotatorPosition() <= 10);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotPreferences.init();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the button bindings
    configureButtonBindings();
    claw.setDefaultCommand(manualClaw);
    swerveDrive.setDefaultCommand(driveWithController);


    // Init Shuffleboard
    RobotPreferences.addShuffleBoardTab();
    swerveDrive.initShuffleboardTab();
    raspberryPiVision.addShuffleboardTab();
    arm.addShuffleboardTab();
    Autonomous.addAutonomousShuffleboardTab();

    if (ClimberRotator.enableTab.getValue()) {
      ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
      // climberRotator.addShuffleboardLayout(climberTab);
      climberHooks.addShuffleboardLayout(climberTab);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
   * then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverButtonA.whenPressed(new InstantCommand(() -> {}, allSubsystems) ); // Interrupt all
    driverMenuButton.whenPressed(new InstantCommand(() -> swerveDrive.resetHeading()));
    // driverLeftBumper.whenPressed(new KeepClimberRotatorVertical(climberRotator));

    // driverButtonB.whenPressed(new DriveStraight(swerveDrive, .25, 0)); // testing
    // driverButtonY.whenPressed(new DriveStraight(swerveDrive, .25, 180)); // testing

    driverDpadUp.whenPressed(new TurnToAngle(swerveDrive, 135));
    driverDpadRight.whenPressed(new TurnToAngle(swerveDrive, 45));
    driverDpadDown.whenPressed(new TurnToAngle(swerveDrive, -45));
    driverDpadLeft.whenPressed(new TurnToAngle(swerveDrive, -135));

    manipulatorLeftBumper.whenPressed(armToResting);
    manipulatorRightBumper.whenPressed(armToScoring);
    manipulatorStartButton.whenPressed(new ManualClimber(climberRotator, manipulatorController));
    manipulatorDpadRight.whenPressed(new RotateArmToStowed(arm));
    manipulatorDpadLeft.whenPressed(new RotateArmToScoring2(arm));
    manipulatorButtonA.whenPressed(new RotateArmToScoring2(arm).andThen(new AutoClaw(0.5, 1.0, claw), new RotateArmToResting(RobotContainer.arm))); // previous argument was arm.

    // manipulatorDpadLeft.whenPressed(new DriveStraight(swerveDrive, .2, -90)); // testing
    manipulatorDpadUp.whenHeld(new InstantCommand(() -> climberRotator.rotateMotor(), climberRotator));
    manipulatorDpadUp.whenReleased(new InstantCommand(() -> climberRotator.stopMotor()));
    manipulatorDpadDown.whenHeld(new InstantCommand(() -> climberRotator.backDriveMotor(), climberRotator));
    manipulatorDpadDown.whenReleased(new InstantCommand(() -> climberRotator.stopMotor()));
    // manipulatorButtonA.whenPressed(climbSequencePart1);
    // manipulatorButtonB.whenPressed(climbSequencePart2);
    // manipulatorButtonX.whenPressed(climbSequencePart3);
    // manipulatorButtonY.whenPressed(abortClimb);
    manipulatorMenuButton.whenPressed(new ToggleClimberExtender(climberExtender));

  }

  public void initSubsystems() {
    raspberryPiVision.initPipeline();
  }

  public void stopAllMotors() {
    swerveDrive.stopMotors();
    // climberRotator.stopMotor();
    claw.stopMotor();
  }
}
