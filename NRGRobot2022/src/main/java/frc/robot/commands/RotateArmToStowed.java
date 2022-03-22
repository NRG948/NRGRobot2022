package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RotateArmToStowed extends CommandBase {
    
    private Arm arm;

    public RotateArmToStowed(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("\nStowing Arm\n");
        arm.setGoal(Math.toRadians(Arm.stowedAngle.getValue()));
        arm.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // arm.disable();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return arm.isAtStowedPosition();
    }
}
