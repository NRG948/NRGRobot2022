// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Om WAS HERE TEST COMMIT!!

package frc.robot;

import java.util.List;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.CharacterizeSwerveDrive;
import frc.robot.commands.AutoClaw;
import frc.robot.commands.CharacterizeArm;
import frc.robot.commands.CommandUtils;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.Interrupt;
import frc.robot.commands.KeepClimberRotatorVertical;
import frc.robot.commands.ManualClaw;
import frc.robot.commands.ResetSubsystems;
import frc.robot.commands.RotateArmToResting;
import frc.robot.commands.RotateArmToScoring;
import frc.robot.commands.RotateArmToStowed;
import frc.robot.commands.SetModuleState;
import frc.robot.commands.ToggleClimberExtender;
import frc.robot.commands.TurnToAngle;
import frc.robot.preferences.RobotPreferences;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.BooleanValue;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClimberExtender;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.ClimberRotator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Autonomous", column = 4, row = 3, width = 2, height = 1)
public class RobotContainer {

  public static Translation2d ROBOT_FRONT_LEFT_LOCATION = new Translation2d(0.521, 0.432);
  public static Translation2d ROBOT_FRONT_RIGHT_LOCATION = new Translation2d(0.521, -0.432);

  public static Rotation2d TARMAC_DOWN_ORIENTATION = Rotation2d.fromDegrees(-21);
  public static Rotation2d TARMAC_RIGHT_ORIENTATION = Rotation2d.fromDegrees(69);

  public static Translation2d TARGET_RIGHT_LOCATION = new Translation2d(7.583, 0.594);
  public static Pose2d TARGET_RIGHT_POSE = new Pose2d(TARGET_RIGHT_LOCATION, Rotation2d.fromDegrees(-90));

  // Initial position for Tarmac right, right-side start.
  // TODO: Include adjust for bumper offset from wheel.
  public static Translation2d RIGHT_TARMAC_RIGHT_START_LOCATION = new Translation2d(8.52, 3.04)
      .minus(ROBOT_FRONT_RIGHT_LOCATION.rotateBy(TARMAC_RIGHT_ORIENTATION));
  public static Pose2d RIGHT_TARMAC_RIGHT_START_POSE = new Pose2d(RIGHT_TARMAC_RIGHT_START_LOCATION,
      TARMAC_RIGHT_ORIENTATION);

  @RobotPreferencesValue
  public static BooleanValue enableTesting = new BooleanValue("Autonomous", "enableTesting", false);

  // Operator interface (e.g. Joysticks)
  private final XboxController driveController = new XboxController(2);
  private JoystickButton xboxButtonA = new JoystickButton(driveController, 1); // A Button
  private JoystickButton xboxButtonB = new JoystickButton(driveController, 2); // B Button
  private JoystickButton xboxButtonx = new JoystickButton(driveController, 3); // x Button
  private JoystickButton xboxButtonY = new JoystickButton(driveController, 4); // y Button
  private JoystickButton xboxLeftBumper = new JoystickButton(driveController, 5);
  private JoystickButton xboxRightBumper = new JoystickButton(driveController, 6);
  // Left Middle Button
  private JoystickButton xboxStartButton = new JoystickButton(driveController, 7);
  // Right Middle Button
  private JoystickButton xboxMenuButton = new JoystickButton(driveController, 8);

  private POVButton xboxDpadUp = new POVButton(driveController, 0);
  private POVButton xboxDpadRight = new POVButton(driveController, 90);
  private POVButton xboxDpadDown = new POVButton(driveController, 180);
  private POVButton xboxDpadLeft = new POVButton(driveController, 270);

  // Manipulate interface
  private final XboxController manipulatorController = new XboxController(3);
  private JoystickButton manipulatorButtonA = new JoystickButton(manipulatorController, 1); // A Button
  private JoystickButton manipulatorButtonB = new JoystickButton(manipulatorController, 2); // B Button
  private JoystickButton manipulatorButtonx = new JoystickButton(manipulatorController, 3); // x Button
  private JoystickButton manipulatorButtonY = new JoystickButton(manipulatorController, 4); // y Button
  private JoystickButton manipulatorLeftBumper = new JoystickButton(manipulatorController, 5);
  private JoystickButton manipulatorRightBumper = new JoystickButton(manipulatorController, 6);
  private JoystickButton manipulatorStartButton = new JoystickButton(manipulatorController, 7);

  // Subsystems
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final RaspberryPiVision raspberryPiVision = new RaspberryPiVision();
  private final Claw claw = new Claw(1); // Port 1
  private final Arm arm = new Arm(); // limit switch channels to be updated
  private final ClimberExtender climberExtender = new ClimberExtender();
  private final ClimberHooks climberHooks = new ClimberHooks();
  private final ClimberRotator climberRotator = new ClimberRotator();

  // Commands
  private final DriveWithController driveWithController = new DriveWithController(swerveDrive, driveController);
  private final Interrupt interrupt = new Interrupt(swerveDrive);
  private final DriveForward driveForward = new DriveForward(swerveDrive);
  private final SetModuleState setModuleState_0 = new SetModuleState(swerveDrive, driveController, 0);
  private final SetModuleState setModuleState_90 = new SetModuleState(swerveDrive, driveController, 90);
  private final ManualClaw manualClaw = new ManualClaw(claw, manipulatorController);
  private final RotateArmToResting armToResting = new RotateArmToResting(arm);
  private final RotateArmToScoring armToScoring = new RotateArmToScoring(arm);
  // private final ManualClimber manualClimber = new ManualClimber(climber,
  // driveController);

  private SendableChooser<ChooseAutoPath> chooseAutoPath;
  private SendableChooser<ChooseAutoDelay> chooseAutoDelay;

  private enum ChooseAutoPath {
    NONE,
    PROFILE_DRIVE,
    PROFILE_ARM,
    TEST_DRIVE,
    RIGHT_TARMAC_RIGHT_START,
    RIGHT_TARMAC_LEFT_START,
    DOWN_TARMAC_RIGHT_START,
    DOWN_TARMAC_LEFT_START,
    RIGHT_TARMAC_SHOOT_BACKUP
  }

  private enum ChooseAutoDelay {
    NO_DELAY(0),
    DELAY_2_SECONDS(2),
    DELAY_5_SECONDS(3);

    private double delay;

    ChooseAutoDelay(double delay) {
      this.delay = delay;
    }

    public double getDelay() {
      return delay;
    }
  }

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
    this.addAutonomousShuffleboardTab();
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
    // xboxButtonx.whenPressed(driveWithController);
    xboxButtonA.whenPressed(interrupt);
    // xboxButtonB.whenPressed(setModuleState_0);
    // xboxButtonY.whenPressed(setModuleState_90);
    xboxMenuButton.whenPressed(new InstantCommand(() -> swerveDrive.resetHeading()));

    xboxDpadUp.whenPressed(new TurnToAngle(swerveDrive, 135));
    xboxDpadRight.whenPressed(new TurnToAngle(swerveDrive, 45));
    xboxDpadDown.whenPressed(new TurnToAngle(swerveDrive, -45));
    xboxDpadLeft.whenPressed(new TurnToAngle(swerveDrive, -135));

    manipulatorLeftBumper.whenPressed(armToResting);
    manipulatorRightBumper.whenPressed(armToScoring);

    // xboxMenuButton.whenPressed(interrupt.andThen(manualClimber));
    // xboxButtonB.whenPressed(toggleClimberPiston1);
    // xboxButtonY.whenPressed(toggleClimberPiston2);

    manipulatorButtonA.whenPressed(new ToggleClimberExtender(climberExtender));

    manipulatorStartButton.whileHeld(new KeepClimberRotatorVertical(climberRotator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = getSelectedAutonomousCommand();
    Command delayCommand = getSelectedDelayCommand();

    if (delayCommand == null) {
      return autoCommand;
    } else if (autoCommand == null) {
      return delayCommand;
    } else {
      return delayCommand.andThen(autoCommand);
    }
  }

  /** Returns the autonmous command selected in the Shuffleboard tab. */
  private Command getSelectedAutonomousCommand() {
    switch (chooseAutoPath.getSelected()) {
      case NONE:
        return new InstantCommand(() -> System.out.println("NO AUTONOMOUS COMMAND SELECTED"));

      case PROFILE_DRIVE:
        return new CharacterizeSwerveDrive(swerveDrive);

      case PROFILE_ARM:
        return new CharacterizeArm(arm);

      case TEST_DRIVE:
        return new ResetSubsystems(swerveDrive).andThen(
            CommandUtils.newFollowWaypointsCommand(swerveDrive,
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(-1, -0.25)),
                new Pose2d(-2, 0, Rotation2d.fromDegrees(-180)),
                true),
            new InstantCommand(() -> swerveDrive.stopMotors()));

      case RIGHT_TARMAC_RIGHT_START:
        return new ResetSubsystems(swerveDrive).andThen(
            new InstantCommand(() -> swerveDrive.resetOdometry(RIGHT_TARMAC_RIGHT_START_POSE)), 
            CommandUtils.newFollowWaypointsCommand(swerveDrive,
                RIGHT_TARMAC_RIGHT_START_POSE,
                List.of(new Translation2d(7.684, 1.662)),
                TARGET_RIGHT_POSE,
                true),
            new WaitCommand(1.0),
            CommandUtils.newFutureFollowWaypointsCommand(swerveDrive,
                List.of(new Translation2d(7.684, 1.662)),
                RIGHT_TARMAC_RIGHT_START_POSE,
                true),
            new InstantCommand(() -> swerveDrive.stopMotors()));

      case RIGHT_TARMAC_SHOOT_BACKUP:
        return new ResetSubsystems(swerveDrive).andThen(
            new InstantCommand(() -> swerveDrive.resetOdometry(RIGHT_TARMAC_RIGHT_START_POSE)), 
            new RotateArmToScoring(arm),
            new AutoClaw(1.0, 1, claw),
            new RotateArmToStowed(arm),
            CommandUtils.newFollowWaypointsCommand(swerveDrive,
                RIGHT_TARMAC_RIGHT_START_POSE,
                List.of(new Translation2d(7.684, 1.662)),
                TARGET_RIGHT_POSE,
                true),
            new InstantCommand(() -> swerveDrive.stopMotors()));

      default:
        return null;

    }
  }

  /**
   * Returns a command to wait the period of time selected in the Shuffleboard
   * tab, or null if no delay is selected.
   */
  private Command getSelectedDelayCommand() {
    ChooseAutoDelay delayChoice = chooseAutoDelay.getSelected();

    if (delayChoice.getDelay() == 0) {
      return null;
    }

    return new WaitCommand(delayChoice.getDelay());
  }

  public void initSubsystems() {
    raspberryPiVision.initPipeline();
  }

  private void addAutonomousShuffleboardTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    ShuffleboardLayout autoLayout = autoTab.getLayout("Autonomous", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(6, 4);

    chooseAutoPath = new SendableChooser<ChooseAutoPath>();

    chooseAutoPath.setDefaultOption("None", ChooseAutoPath.NONE);

    if (enableTesting.getValue()) {
      chooseAutoPath.addOption("Profile Drive", ChooseAutoPath.PROFILE_DRIVE);
      chooseAutoPath.addOption("Profile Arm", ChooseAutoPath.PROFILE_ARM);
      chooseAutoPath.addOption("Test Drive", ChooseAutoPath.TEST_DRIVE);
    }

    chooseAutoPath.addOption("Right Tarmac Right Start", ChooseAutoPath.RIGHT_TARMAC_RIGHT_START);
    chooseAutoPath.addOption("Right Tamrac Left Start", ChooseAutoPath.RIGHT_TARMAC_LEFT_START);
    chooseAutoPath.addOption("Down Tarmac Right Start", ChooseAutoPath.DOWN_TARMAC_RIGHT_START);
    chooseAutoPath.addOption("Down Tarmac Left Start", ChooseAutoPath.DOWN_TARMAC_LEFT_START);
    autoLayout.add("AutoPath", chooseAutoPath).withWidget(BuiltInWidgets.kComboBoxChooser);

    chooseAutoDelay = new SendableChooser<ChooseAutoDelay>();
    chooseAutoDelay.setDefaultOption("0 sec", ChooseAutoDelay.NO_DELAY);
    chooseAutoDelay.addOption("2 sec", ChooseAutoDelay.DELAY_2_SECONDS);
    chooseAutoDelay.addOption("5 sec", ChooseAutoDelay.DELAY_5_SECONDS);
    autoLayout.add("Delay", chooseAutoDelay).withWidget(BuiltInWidgets.kComboBoxChooser);
  }
}
