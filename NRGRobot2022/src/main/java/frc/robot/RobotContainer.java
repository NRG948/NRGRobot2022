// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Om WAS HERE TEST COMMIT!!

package frc.robot;

import static frc.robot.subsystems.ClimberHooks.HookSelection.HOOK_1;
import static frc.robot.subsystems.ClimberHooks.HookSelection.HOOK_2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.Interrupt;
import frc.robot.commands.KeepClimberRotatorVertical;
import frc.robot.commands.ManualClaw;
import frc.robot.commands.ManualClimber;
import frc.robot.commands.RotateArmToResting;
import frc.robot.commands.RotateArmToScoring;
import frc.robot.commands.RotateClimber;
import frc.robot.commands.SetHook;
import frc.robot.commands.ToggleClimberExtender;
import frc.robot.commands.TurnToAngle;
import frc.robot.preferences.RobotPreferences;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClimberExtender;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.ClimberHooks.State;
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
  private JoystickButton driverButtonX = new JoystickButton(driveController, 3); // x Button
  private JoystickButton driverButtonY = new JoystickButton(driveController, 4); // y Button
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
  private JoystickButton manipulatorButtonX = new JoystickButton(manipulatorController, 3); // x Button
  private JoystickButton manipulatorButtonY = new JoystickButton(manipulatorController, 4); // y Button
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
  public static final Arm arm = new Arm(); // limit switch channels to be updated
  public static final ClimberExtender climberExtender = new ClimberExtender();
  public static final ClimberHooks climberHooks = new ClimberHooks();
  public static final ClimberRotator climberRotator = new ClimberRotator();

  // Commands
  private final DriveWithController driveWithController = new DriveWithController(swerveDrive, driveController);
  private final Interrupt interrupt = new Interrupt(swerveDrive);
  private final ManualClaw manualClaw = new ManualClaw(claw, manipulatorController);
  private final RotateArmToResting armToResting = new RotateArmToResting(arm);
  private final RotateArmToScoring armToScoring = new RotateArmToScoring(arm);
  private final ManualClimber manualClimber = new ManualClimber(climberRotator, manipulatorController);

    /*
  CommandSequence:
  1: 
  Hook 1: Open
  Hook 2: Open
  Hits first bar/(Hits Limit switch 1): 
    Hook 1: Closed
  2: 
  Hook 2: Retracted
  Hits second bar (Hits Limit switch 2): 
    Extend Hook 1
    Retract Hook 1
  3: 
  Hook 1: Retracted
  P2: Retracted
  Hits Traveral Bar (Hits Limit switch 1): 
    Extend Hook 2
  Stop Motor. 
  */

  private final SequentialCommandGroup climbSequencePart1 = 
    new ToggleClimberExtender(climberExtender)
      .andThen(new SetHook(climberHooks, HOOK_1, State.OPEN))
      .andThen(new WaitCommand(1.0))
      .andThen(new DriveStraight(swerveDrive, .3, Math.toRadians(0))) //Drive slowly to the bar
      .until(() -> climberHooks.isBarDetected(HOOK_1))
      .andThen(new SetHook(climberHooks, HOOK_1, State.CLOSED))
    ;

    private final SequentialCommandGroup climbSequencePart2 = 
      new SetHook(climberHooks, HOOK_2, State.OPEN)
      .andThen(new RotateClimber(climberRotator)
      .until(() -> climberHooks.isBarDetected(HOOK_2)))
      .andThen(new SetHook(climberHooks, HOOK_2, State.CLOSED))
      .andThen(new WaitCommand(1.0))
      ;

      private final SequentialCommandGroup climbSequencePart3 = 
      new SetHook(climberHooks, HOOK_1, State.OPEN)
      .andThen(new RotateClimber(climberRotator)
      .until(() -> climberHooks.isBarDetected(HOOK_1)))
      .andThen(new SetHook(climberHooks, HOOK_1, State.CLOSED))
      .andThen(new WaitCommand(1.0))
      .andThen(new SetHook(climberHooks, HOOK_2, State.OPEN))
      .andThen(new InstantCommand(() -> climberRotator.stopMotor()))
      ;

      

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
      climberRotator.addShuffleboardLayout(climberTab);
      climberHooks.addShuffleboardLayout(climberTab);
    }
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
    driverButtonA.whenPressed(interrupt);
    driverMenuButton.whenPressed(new InstantCommand(() -> swerveDrive.resetHeading()));

    driverDpadUp.whenPressed(new TurnToAngle(swerveDrive, 135));
    driverDpadRight.whenPressed(new TurnToAngle(swerveDrive, 45));
    driverDpadDown.whenPressed(new TurnToAngle(swerveDrive, -45));
    driverDpadLeft.whenPressed(new TurnToAngle(swerveDrive, -135));

    manipulatorLeftBumper.whenPressed(armToResting);
    manipulatorRightBumper.whenPressed(armToScoring);
    manipulatorDpadUp.whenPressed(new SetHook(climberHooks, HOOK_1, State.CLOSED));
    manipulatorDpadUp.whenReleased(new SetHook(climberHooks, HOOK_1, State.OPEN));
    manipulatorDpadDown.whenPressed(new SetHook(climberHooks, HOOK_2, State.CLOSED));
    manipulatorDpadDown.whenReleased(new SetHook(climberHooks, HOOK_2, State.OPEN));
    manipulatorMenuButton.whenPressed(manualClimber);
    manipulatorDpadRight.whenPressed(new DriveStraight(swerveDrive, .2, Math.toRadians(180))); // testing

    manipulatorButtonA.whenPressed(climbSequencePart1);
    manipulatorButtonB.whenPressed(climbSequencePart2);
    manipulatorButtonX.whenPressed(climbSequencePart3);
    manipulatorButtonY.whenPressed(new ToggleClimberExtender(climberExtender));

    manipulatorStartButton.whileHeld(new KeepClimberRotatorVertical(climberRotator));
  }

  public void initSubsystems() {
    raspberryPiVision.initPipeline();
  }
}
