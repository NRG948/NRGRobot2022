package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.Pneumatics;

public class ClimberHooks extends SubsystemBase {

    private final DigitalInput beamBreak1;
    private final DigitalInput beamBreak2;
    private boolean beamBreak1Previous = false; // Value of beamBreak1 last cycle
    private boolean beamBreak2Previous = false;

    
    public enum HookSelection {
        HOOK_1, HOOK_2;
    }

    /** Creates a new ClimberHooks subsystem. **/
    public ClimberHooks() {

        // The beam breaks will read TBD(true/false) when it engages the bar
        beamBreak1 = new DigitalInput(6);
        beamBreak2 = new DigitalInput(5);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        beamBreak1Previous = beamBreak1.get();
        beamBreak2Previous = beamBreak2.get();

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /** Returns true iff the climber hook is latched on a bar. */
    public boolean isBarDetected(HookSelection hook) {
        if (hook.equals(HookSelection.HOOK_1)) {
            return !beamBreak1.get();
        } else {
            return !beamBreak2.get();
        }
    }
    public boolean isPassedBar(HookSelection hook){
        if (hook.equals(HookSelection.HOOK_1)) {
            return beamBreak1Previous == true && beamBreak1.get() == false;
        } else {
            return beamBreak2Previous == true && beamBreak2.get() == false;
        }
    }

    public void addShuffleboardLayout(ShuffleboardTab climberTab) {
        ShuffleboardLayout rotatorLayout = climberTab.getLayout("Hook", BuiltInLayouts.kGrid)
                .withPosition(1, 0)
                .withSize(1, 2);

        rotatorLayout.addBoolean("Beam Break 1", () -> !beamBreak1.get()).withWidget(BuiltInWidgets.kBooleanBox);
        rotatorLayout.addBoolean("Beam Break 2", () -> !beamBreak2.get()).withWidget(BuiltInWidgets.kBooleanBox);
    }
}