// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ManualClaw extends CommandBase {
    private Claw claw;
    private XboxController xboxController;
    
    public ManualClaw(Claw claw, XboxController xboxController) {
        this.claw = claw;
        this.xboxController = xboxController;
        addRequirements(claw);
    }

    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        System.out.println("\nManual Claw\n");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        claw.activateClaw(-xboxController.getRightY());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
