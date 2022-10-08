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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoClaw;
import frc.robot.commands.CharacterizeArm;
import frc.robot.commands.CharacterizeSwerveDrive;
import frc.robot.commands.CommandUtils;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.DriveStraightTo;
import frc.robot.commands.ResetSubsystems;
import frc.robot.commands.RotateArmToResting;
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
        RIGHT_TARMAC_TWO_BALLS,
        RIGHT_TARMAC_TWO_BALLS_FAST,
        RIGHT_TARMAC_THREE_BALL,
        DOWN_TARMAC_TWO_BALLS,
        RIGHT_TARMAC_SHOOT_BACKUP,
        DOWN_TARMAC_SHOOT_BACKUP,
        DOWN_TARMAC_DRIVE_BACKWARDS,
        RIGHT_TARMAC_DRIVE_BACKWARDS
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

    // Testing Positions for 3 ball -> 
    private static Translation2d ROBOT_FRONT_FIRST_POINT = new Translation2d(8.269, 2.787);
    private static Translation2d ROBOT_FRONT_SECOND_POINT = new Translation2d(7.606-0.40, 0.279-.39); //.279 changes 0.20 to 0.40 
    private static Translation2d ROBOT_FRONT_THIRD_POINT = new Translation2d(7.864, 3.068);
    private static Translation2d ROBOT_FRONT_FOURTH_POINT = new Translation2d(4.125, 1.887); 
    private static Translation2d ROBOT_FRONT_FIFTH_POINT = new Translation2d(7.493, 3.383);


    //rotations
    private static Rotation2d TARMAC_FIRST_ORIENTATION = Rotation2d.fromDegrees(160);
    private static Rotation2d TARMAC_SECOND_ORIENTATION = Rotation2d.fromDegrees(40); //changed from 40 to 43
    private static Rotation2d TARMAC_THIRD_ORIENTATION = Rotation2d.fromDegrees(12.68); // changed degrees 
    private static Rotation2d TARMAC_FOURTH_ORIENTATION = Rotation2d.fromDegrees(32.21); //changed to positive
    private static Rotation2d TARMAC_FIFTH_ORIENTATION = Rotation2d.fromDegrees(-32.21);

    //poses
    private static Pose2d WAYPOINT_ONE = new Pose2d(ROBOT_FRONT_FIRST_POINT,
            TARMAC_FIRST_ORIENTATION);
    private static Pose2d WAYPOINT_TWO = new Pose2d(ROBOT_FRONT_SECOND_POINT,
            TARMAC_SECOND_ORIENTATION);
    private static Pose2d WAYPOINT_THREE = new Pose2d(ROBOT_FRONT_THIRD_POINT,
            TARMAC_THIRD_ORIENTATION);
    private static Pose2d WAYPOINT_FOUR = new Pose2d(ROBOT_FRONT_FOURTH_POINT,
            TARMAC_FOURTH_ORIENTATION);
    private static Pose2d WAYPOINT_FIVE = new Pose2d(ROBOT_FRONT_FIFTH_POINT,
    TARMAC_FIFTH_ORIENTATION);


    //
    private static Translation2d ROBOT_FRONT_LEFT_LOCATION = new Translation2d(0.521, 0.432);
    private static Translation2d ROBOT_FRONT_RIGHT_LOCATION = new Translation2d(0.521, -0.432);
    private static Translation2d ROBOT_THREE_BALL_TEST = new Translation2d(5.12, 1.887);

    private static Rotation2d TARMAC_DOWN_ORIENTATION = Rotation2d.fromDegrees(-21);
    private static Rotation2d TARMAC_RIGHT_ORIENTATION = Rotation2d.fromDegrees(100);
    private static Rotation2d ROBOT_THREE_BALL_TEST_ORIENTATION = Rotation2d.fromDegrees(240);

    // Initial position for Tarmac right, right-side start.
    private static Translation2d RIGHT_TARMAC_RIGHT_START_LOCATION = new Translation2d(8.52, 3.04)
            .minus(ROBOT_FRONT_RIGHT_LOCATION.rotateBy(TARMAC_RIGHT_ORIENTATION));
    private static Pose2d RIGHT_TARMAC_RIGHT_START_POSE = new Pose2d(RIGHT_TARMAC_RIGHT_START_LOCATION,
            TARMAC_RIGHT_ORIENTATION);
    private static Pose2d ROBOT_THREE_BALL = new Pose2d(ROBOT_THREE_BALL_TEST,
            ROBOT_THREE_BALL_TEST_ORIENTATION);
    private static Translation2d RIGHT_TARMAC_RIGHT_WAYPOINT = new Translation2d(7.684, 1.662);

    private static Translation2d DOWN_TARMAC_LEFT_START_LOCATION = new Translation2d(7.651, 4.957)
            .minus(ROBOT_FRONT_LEFT_LOCATION.rotateBy(TARMAC_DOWN_ORIENTATION));
    private static Pose2d DOWN_TARMAC_LEFT_START_POSE = new Pose2d(DOWN_TARMAC_LEFT_START_LOCATION,
            TARMAC_DOWN_ORIENTATION);

    private static Translation2d TARGET_RIGHT_LOCATION = new Translation2d(7.583, 0.694);
    private static Pose2d TARGET_RIGHT_POSE = new Pose2d(TARGET_RIGHT_LOCATION, Rotation2d.fromDegrees(-100));

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

            case RIGHT_TARMAC_TWO_BALLS:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        new InstantCommand(
                                () -> RobotContainer.swerveDrive.resetOdometry(RIGHT_TARMAC_RIGHT_START_POSE)),
                        new RotateArmToScoring(RobotContainer.arm),
                        new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                        new AutoClaw(0.75, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm),
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                RIGHT_TARMAC_RIGHT_START_POSE,
                                List.of(RIGHT_TARMAC_RIGHT_WAYPOINT),
                                TARGET_RIGHT_POSE,
                                true)
                                .alongWith(new WaitUntilCommand(
                                        () -> RobotContainer.swerveDrive.getHeadingDegrees() <= 15.0)
                                                .andThen(new RotateArmToResting(RobotContainer.arm))
                                                .andThen(() -> RobotContainer.claw.activateClaw(-1.0))),
                        new InstantCommand(() -> RobotContainer.claw.stopMotor()),
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                TARGET_RIGHT_POSE,
                                List.of(RIGHT_TARMAC_RIGHT_WAYPOINT),
                                RIGHT_TARMAC_RIGHT_START_POSE,
                                true)
                                .alongWith(new RotateArmToScoring(RobotContainer.arm)),
                        new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                        new AutoClaw(0.75, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm));
            //

            case RIGHT_TARMAC_THREE_BALL:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        new InstantCommand(
                                () -> RobotContainer.swerveDrive.resetOdometry(RIGHT_TARMAC_RIGHT_START_POSE)),
                        new RotateArmToScoring(RobotContainer.arm),
                        new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                        new AutoClaw(0.75, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm),
                        new DriveStraightTo(RobotContainer.swerveDrive, 0.4, WAYPOINT_TWO)
                                .alongWith(new WaitUntilCommand(
                                        () -> RobotContainer.swerveDrive.getHeadingDegrees() >= 120.0)
                                                .andThen(new RotateArmToResting(RobotContainer.arm)
                                                        .alongWith(new InstantCommand(() -> RobotContainer.claw.activateClaw(-1.0))))),
                        new InstantCommand(() -> RobotContainer.claw.stopMotor()),
                        new WaitCommand(0.5),
                        new DriveStraightTo(RobotContainer.swerveDrive, 0.4, WAYPOINT_THREE)
                                .alongWith(new RotateArmToScoring(RobotContainer.arm)),
                        new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                        new AutoClaw(0.75, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm),
                        new DriveStraightTo(RobotContainer.swerveDrive, 0.4, WAYPOINT_FOUR)
                                .alongWith(new WaitUntilCommand(
                                        () -> RobotContainer.swerveDrive.getHeadingDegrees() >= 120.0)
                                                .andThen(new RotateArmToResting(RobotContainer.arm)
                                                        .alongWith(new InstantCommand(() -> RobotContainer.claw.activateClaw(-1.0))))),
                        new WaitCommand(0.15),  
                        new RotateArmToScoring(RobotContainer.arm),
                        new DriveStraightTo(RobotContainer.swerveDrive, 0.4, WAYPOINT_FIVE)
                                .andThen()
                                );

                        //
                case RIGHT_TARMAC_TWO_BALLS_FAST:
                        return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                                new InstantCommand(
                                        () -> RobotContainer.swerveDrive.resetOdometry(RIGHT_TARMAC_RIGHT_START_POSE)),
                                new RotateArmToScoring(RobotContainer.arm),
                                new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                                new AutoClaw(0.75, 1, RobotContainer.claw),
                                new RotateArmToStowed(RobotContainer.arm),
                                new DriveStraightTo(RobotContainer.swerveDrive, 0.4, TARGET_RIGHT_POSE)
                                        .alongWith(new WaitUntilCommand(
                                                () -> RobotContainer.swerveDrive.getHeadingDegrees() <= 15.0)
                                                        .andThen(new RotateArmToResting(RobotContainer.arm)
                                                                .alongWith(new InstantCommand(() -> RobotContainer.claw.activateClaw(-1.0))))),
                                new InstantCommand(() -> RobotContainer.claw.stopMotor()),
                                new WaitCommand(0.5),
                                new DriveStraightTo(RobotContainer.swerveDrive, 0.3, RIGHT_TARMAC_RIGHT_START_POSE)
                                        .alongWith(new RotateArmToScoring(RobotContainer.arm)),
                                new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                                new AutoClaw(0.75, 1, RobotContainer.claw),
                                new RotateArmToStowed(RobotContainer.arm));

            case DOWN_TARMAC_TWO_BALLS:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        new InstantCommand(
                                () -> RobotContainer.swerveDrive.resetOdometry(DOWN_TARMAC_LEFT_START_POSE)),
                        new RotateArmToScoring(RobotContainer.arm),
                        new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                        new AutoClaw(0.75, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm),
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                DOWN_TARMAC_LEFT_START_POSE,
                                List.of(new Translation2d(5.919, 5.362)),
                                TARGET_DOWN_POSE,
                                true)
                                .alongWith(new WaitUntilCommand(
                                        () -> RobotContainer.swerveDrive.getHeadingDegrees() <= -45.0)
                                                .andThen(new RotateArmToResting(RobotContainer.arm))
                                                .andThen(() -> RobotContainer.claw.activateClaw(-1.0))),
                        new InstantCommand(() -> RobotContainer.claw.stopMotor()),
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                TARGET_DOWN_POSE,
                                List.of(new Translation2d(5.919, 5.362)),
                                DOWN_TARMAC_LEFT_START_POSE,
                                true)
                                .alongWith(new RotateArmToScoring(RobotContainer.arm)),
                        new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                        new AutoClaw(0.75, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm));

            case RIGHT_TARMAC_SHOOT_BACKUP:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        new InstantCommand(
                                () -> RobotContainer.swerveDrive.resetOdometry(RIGHT_TARMAC_RIGHT_START_POSE)),
                        new RotateArmToScoring(RobotContainer.arm),
                        new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                        new AutoClaw(1.0, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm),
                        CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                RIGHT_TARMAC_RIGHT_START_POSE,
                                List.of(new Translation2d(7.684, 1.662)),
                                new Pose2d(new Translation2d(8.0, 0.594), Rotation2d.fromDegrees(-90)),
                                true),
                        new InstantCommand(() -> RobotContainer.swerveDrive.stopMotors()));

            case DOWN_TARMAC_SHOOT_BACKUP:
                return new ResetSubsystems(RobotContainer.swerveDrive).andThen(
                        new InstantCommand(
                                () -> RobotContainer.swerveDrive.resetOdometry(DOWN_TARMAC_LEFT_START_POSE)),
                        new RotateArmToScoring(RobotContainer.arm),
                        new WaitUntilCommand(() -> RobotContainer.arm.isAtScoringPosition()).withTimeout(0.5),
                        new AutoClaw(1.0, 1, RobotContainer.claw),
                        new RotateArmToStowed(RobotContainer.arm)
                       // CommandUtils.newFollowWaypointsCommand(RobotContainer.swerveDrive,
                                // DOWN_TARMAC_LEFT_START_POSE,
                                // List.of(new Translation2d(5.919, 5.362)),
                                // TARGET_DOWN_POSE,
                                // true),
                                .andThen(new DriveStraightDistance(RobotContainer.swerveDrive, 0.33, -180, 1.5)), //reversed sign for -180


                        new InstantCommand(() -> RobotContainer.swerveDrive.stopMotors()
                        ));

            case DOWN_TARMAC_DRIVE_BACKWARDS:
                return new ResetSubsystems(RobotContainer.swerveDrive)
                        .andThen(new DriveStraightDistance(RobotContainer.swerveDrive, 0.33, 180, 3)); //reversed sign for 180 and changed distance to 3 from 1.5

            case RIGHT_TARMAC_DRIVE_BACKWARDS:
                return new ResetSubsystems(RobotContainer.swerveDrive)
                        .andThen(() -> RobotContainer.swerveDrive
                                .resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(45))))
                        .andThen(new DriveStraightDistance(RobotContainer.swerveDrive, 0.33, 225, 1));

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

        chooseAutoPath.addOption("Right Tarmac Two Balls", ChooseAutoPath.RIGHT_TARMAC_TWO_BALLS);
        chooseAutoPath.addOption("Right Tarmac Two Balls (Fast)", ChooseAutoPath.RIGHT_TARMAC_TWO_BALLS_FAST);
        chooseAutoPath.addOption("Right Tarmac Three Ball", ChooseAutoPath.RIGHT_TARMAC_THREE_BALL);
        chooseAutoPath.addOption("Down Tarmac Two Balls", ChooseAutoPath.DOWN_TARMAC_TWO_BALLS);
        chooseAutoPath.addOption("Right Tarmac Shoot & Backup", ChooseAutoPath.RIGHT_TARMAC_SHOOT_BACKUP);
        chooseAutoPath.addOption("Down Tarmac Shoot & Backup", ChooseAutoPath.DOWN_TARMAC_SHOOT_BACKUP);
        chooseAutoPath.addOption("Right Tarmac Drive Straight Backwards", ChooseAutoPath.RIGHT_TARMAC_DRIVE_BACKWARDS);
        chooseAutoPath.addOption("Down Tarmac Drive Straight Backwards", ChooseAutoPath.DOWN_TARMAC_DRIVE_BACKWARDS);
        autoLayout.add("AutoPath", chooseAutoPath).withWidget(BuiltInWidgets.kComboBoxChooser);

        chooseAutoDelay.setDefaultOption("0 sec", ChooseAutoDelay.NO_DELAY);
        chooseAutoDelay.addOption("2 sec", ChooseAutoDelay.DELAY_2_SECONDS);
        chooseAutoDelay.addOption("5 sec", ChooseAutoDelay.DELAY_5_SECONDS);
        autoLayout.add("Delay", chooseAutoDelay).withWidget(BuiltInWidgets.kComboBoxChooser);
    }
}
