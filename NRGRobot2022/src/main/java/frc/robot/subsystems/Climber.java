// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final PWMVictorSPX climberMotor = new PWMVictorSPX(0);
  private final DoubleSolenoid piston[] = {
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2),
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4)
  };

  public enum Piston {
    _1(0), _2(1);

    private final int index;

    private Piston(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  public enum State {
    EXTEND, RETRACT;
  }

  /** Creates a new Climber. */
  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rotateMotor(double power) {
    climberMotor.set(power);
  }

  /** Returns the current state of the acquirer pistons */
  public State getState(Piston p) {
    return piston[p.getIndex()].get() == Value.kForward ? State.EXTEND : State.RETRACT;
  }

  public void setState(State state, Piston p) {
    piston[p.getIndex()].set(state == State.EXTEND ? Value.kForward : Value.kReverse);
  }

  public void toggleState(Piston p) {
    setState(getState(p) == State.EXTEND ? State.RETRACT : State.EXTEND, p);
  }

    /*
  CommandSequence:
  1: 
  P1: Retracted
  P2: Extended
  Hits first bar/(Hits Limit switch 1): 
    Retract P2
  2: 
  P1: Retracted
  P2: Retracted
  Hits second bar (Hits Limit switch 2): 
    Extend P1
    Retract P1
  3: 
  P1: Retracted
  P2: Retracted
  Hits Traveral Bar (Hits Limit switch 1): 
    Extend P2
  Stop Motor. 
  */

}
