package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RotateArmToResting extends CommandBase {
    
    private Arm arm;

    public RotateArmToResting(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setGoal(0);
        arm.enable();
        if (arm.getLimitSwitchResting()){
            arm.disable();
            arm.resetEncoder();
        } 
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
