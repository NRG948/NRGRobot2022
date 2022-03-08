// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;

public final class CommandUtils {
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 430.35;

    public static Command newFollowWaypointsCommand(
            SwerveDrive swerve,
            Pose2d initialPose2d,
            List<Translation2d> waypoints,
            Pose2d finalPose2d,
            boolean reversed) {
        // Create config for trajectory
        Trajectory trajectory = swerve.generateTrajectory(initialPose2d, waypoints, finalPose2d, reversed);

        var thetaController = new ProfiledPIDController(
                SwerveDrive.turnP.getValue(), 
                SwerveDrive.turnI.getValue(), 
                SwerveDrive.turnD.getValue(), 
                SwerveDrive.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerve::getPose2d, // Functional interface to feed supplier
                swerve.getKinematics(),

                // Position controllers
                new PIDController(SwerveDrive.driveP.getValue(), SwerveDrive.driveI.getValue(), SwerveDrive.driveD.getValue()),
                new PIDController(SwerveDrive.driveP.getValue(), SwerveDrive.driveI.getValue(), SwerveDrive.driveD.getValue()),
                thetaController,
                swerve::setModuleStates,
                swerve);

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> swerve.stopMotors());
    }
}
