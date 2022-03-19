// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoClaw;
import frc.robot.commands.CharacterizeArm;
import frc.robot.commands.CharacterizeSwerveDrive;
import frc.robot.commands.CommandUtils;
import frc.robot.commands.ResetSubsystems;
import frc.robot.commands.RotateArmToScoring;
import frc.robot.commands.RotateArmToStowed;
import frc.robot.preferences.RobotPreferencesLayout;
import frc.robot.preferences.RobotPreferencesValue;
import frc.robot.preferences.RobotPreferences.BooleanValue;

/** Add your docs here. */
@RobotPreferencesLayout(groupName = "Autonomous", column = 4, row = 3, width = 2, height = 1)
public class Autonomous {

    @RobotPreferencesValue
    public static BooleanValue enableTesting = new BooleanValue("Autonomous", "enableTesting", false);

    private static SendableChooser<ChooseAutoPath> chooseAutoPath = new SendableChooser<>();
    private static SendableChooser<ChooseAutoDelay> chooseAutoDelay = new SendableChooser<>();

    private static enum ChooseAutoPath {
        NONE,
        PROFILE_DRIVE,
        PROFILE_ARM,
        TEST_DRIVE,
        RIGHT_TARMAC_RIGHT_START,
        // RIGHT_TARMAC_LEFT_START,
        DOWN_TARMAC_RIGHT_START,
        // DOWN_TARMAC_LEFT_START,
        RIGHT_TARMAC_SHOOT_BACKUP,
        DOWN_TARMAC_SHOOT_BACKUP
    }

    private static enum ChooseAutoDelay {
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

    private static Translation2d ROBOT_FRONT_LEFT_LOCATION = new Translation2d(0.521, 0.432);
    private static Translation2d ROBOT_FRONT_RIGHT_LOCATION = new Translation2d(0.521, -0.432);

    private static Rotation2d TARMAC_DOWN_ORIENTATION = Rotation2d.fromDegrees(-21);
    private static Rotation2d TARMAC_RIGHT_ORIENTATION = Rotation2d.fromDegrees(69);

    // Initial position for Tarmac right, right-side start.
    // TODO: Include adjust for bumper offset from wheel.
    private static Translation2d RIGHT_TARMAC_RIGHT_START_LOCATION = new Translation2d(8.52, 3.04)
            .minus(ROBOT_FRONT_RIGHT_LOCATION.rotateBy(TARMAC_RIGHT_ORIENTATION));
    private static Pose2d RIGHT_TARMAC_RIGHT_START_POSE = new Pose2d(RIGHT_TARMAC_RIGHT_START_LOCATION,
            TARMAC_RIGHT_ORIENTATION);

    private static Translation2d DOWN_TARMAC_LEFT_START_LOCATION = new Translation2d(7.651, 4.957)
            .minus(ROBOT_FRONT_LEFT_LOCATION.rotateBy(TARMAC_DOWN_ORIENTATION));
    private static Pose2d DOWN_TARMAC_LEFT_START_POSE = new Pose2d(DOWN_TARMAC_LEFT_START_LOCATION, TARMAC_DOWN_ORIENTATION);
    private static Translation2d RIGHT_TARMAC_RIGHT_WAYPOINT = new Translation2d(7.684, 1.662);

    private static Translation2d TARGET_RIGHT_LOCATION = new Translation2d(7.583, 0.594);
    private static Pose2d TARGET_RIGHT_POSE = new Pose2d(TARGET_RIGHT_LOCATION, Rotation2d.fromDegrees(-90));
    private static Translation2d TARGET_DOWN_LOCATION = new Translation2d(5.177, 6.105);
    private static Pose2d TARGET_DOWN_POSE = new Pose2d(TARGET_DOWN_LOCATION, Rotation2d.fromDegrees(159));

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public static Command getAutonomousCommand() {
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
    private static Command getSelectedAutonomousCommand() {
        switch (chooseAutoPath.getSelected()) {
            case NONE:
                return new InstantCommand(() -> System.out.println("NO AUTONOMOUS COMMAND SELECTED"));

            case PROFILE_DRIVE:
                return new CharacterizeSwerveDrive(RobotContainer.swerveDrive);

            case PROFILE_ARM:
                return new CharacterizeArm(RobotContainer.arm);

            case TEST_DRIVE:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(new Translation2d(-1, -0.25)),
                                new Pose2d(-2, 0, Rotation2d.fromDegrees(-180)),
                                true),
                        new InstantCommand(() -> RobotContainer.swerveDrive.stopMotors()));

            case RIGHT_TARMAC_RIGHT_START:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        new InstantCommand(
                                () -> RobotContainer.swerveDrive.resetOdometry(RIGHT_TARMAC_RIGHT_START_POSE)),
                        new RotateArmToScoring(RobotContainer.arm),
                        new AutoClaw(1.0, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm),
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                RIGHT_TARMAC_RIGHT_START_POSE,
                                List.of(RIGHT_TARMAC_RIGHT_WAYPOINT),
                                TARGET_RIGHT_POSE,
                                true),
                        new WaitCommand(1.0),
                        CommandUtils.newFutureFollowWaypointsCommand(RobotContainer.swerveDrive,
                                List.of(RIGHT_TARMAC_RIGHT_WAYPOINT),
                                RIGHT_TARMAC_RIGHT_START_POSE,
                                true),
                        new InstantCommand(() -> RobotContainer.swerveDrive.stopMotors()));

            case RIGHT_TARMAC_SHOOT_BACKUP:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        new InstantCommand(
                                () -> RobotContainer.swerveDrive.resetOdometry(RIGHT_TARMAC_RIGHT_START_POSE)),
                        new RotateArmToScoring(RobotContainer.arm),
                        new AutoClaw(1.0, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm),
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                RIGHT_TARMAC_RIGHT_START_POSE,
                                List.of(new Translation2d(7.684, 1.662)),
                                TARGET_RIGHT_POSE,
                                true),
                        new InstantCommand(() -> RobotContainer.swerveDrive.stopMotors()));

            case DOWN_TARMAC_SHOOT_BACKUP:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        new InstantCommand(
                                () -> RobotContainer.swerveDrive.resetOdometry(DOWN_TARMAC_LEFT_START_POSE)),
                        new RotateArmToScoring(RobotContainer.arm),
                        new AutoClaw(1.0, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm),
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                DOWN_TARMAC_LEFT_START_POSE,
                                List.of(new Translation2d(5.919, 5.362)),
                                TARGET_DOWN_POSE,
                                true),
                        new InstantCommand(() -> RobotContainer.swerveDrive.stopMotors()));

            default:
                return null;

        }
    }

    /**
     * Returns a command to wait the period of time selected in the Shuffleboard
     * tab, or null if no delay is selected.
     */
    private static Command getSelectedDelayCommand() {
        ChooseAutoDelay delayChoice = chooseAutoDelay.getSelected();

        if (delayChoice.getDelay() == 0) {
            return null;
        }

        return new WaitCommand(delayChoice.getDelay());
    }

    public static void addAutonomousShuffleboardTab() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

        ShuffleboardLayout autoLayout = autoTab.getLayout("Autonomous", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(6, 4);

        chooseAutoPath.setDefaultOption("None", ChooseAutoPath.NONE);

        if (enableTesting.getValue()) {
            chooseAutoPath.addOption("Profile Drive", ChooseAutoPath.PROFILE_DRIVE);
            chooseAutoPath.addOption("Profile Arm", ChooseAutoPath.PROFILE_ARM);
            chooseAutoPath.addOption("Test Drive", ChooseAutoPath.TEST_DRIVE);
        }

        chooseAutoPath.addOption("Right Tarmac Right Start", ChooseAutoPath.RIGHT_TARMAC_RIGHT_START);
        // chooseAutoPath.addOption("Right Tamrac Left Start",
        // ChooseAutoPath.RIGHT_TARMAC_LEFT_START);
        chooseAutoPath.addOption("Down Tarmac Right Start", ChooseAutoPath.DOWN_TARMAC_RIGHT_START);
        // chooseAutoPath.addOption("Down Tarmac Left Start",
        // ChooseAutoPath.DOWN_TARMAC_LEFT_START);
        chooseAutoPath.addOption("Right Tarmac Shoot & Backup", ChooseAutoPath.RIGHT_TARMAC_SHOOT_BACKUP);
        chooseAutoPath.addOption("Down Tarmac Shoot & Backup", ChooseAutoPath.DOWN_TARMAC_SHOOT_BACKUP);
        autoLayout.add("AutoPath", chooseAutoPath).withWidget(BuiltInWidgets.kComboBoxChooser);

        chooseAutoDelay.setDefaultOption("0 sec", ChooseAutoDelay.NO_DELAY);
        chooseAutoDelay.addOption("2 sec", ChooseAutoDelay.DELAY_2_SECONDS);
        chooseAutoDelay.addOption("5 sec", ChooseAutoDelay.DELAY_5_SECONDS);
        autoLayout.add("Delay", chooseAutoDelay).withWidget(BuiltInWidgets.kComboBoxChooser);
    }
}
