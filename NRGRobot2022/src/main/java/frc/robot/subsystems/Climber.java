// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  
  private final PWMVictorSPX climberMotor; 
  private final DoubleSolenoid piston;
  private final DoubleSolenoid piston2;
  
  public enum State {
    EXTEND, RETRACT;
  } 

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new PWMVictorSPX(0);
    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public DoubleSolenoid getP1() {
    return piston;
  }

  public DoubleSolenoid getP2() {
      return piston2;
  }

  public void rotateMotor(double power) {
      climberMotor.set(power);
  }

  /** Returns the current state of the acquirer pistons */
  public State getState(DoubleSolenoid p) {
      return p.get() == Value.kForward ? State.EXTEND : State.RETRACT;
  }

  public void setState(State state, DoubleSolenoid p) {
      p.set(state == State.EXTEND ? Value.kForward : Value.kReverse);
  }

  public void toggleState(DoubleSolenoid p) {
      setState(getState(p) == State.EXTEND ? State.RETRACT : State.EXTEND, p);
  }


}
