
package frc.robot.subsystems;

import javax.print.attribute.standard.RequestingUserName;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends ProfiledPIDSubsystem {
    private final DigitalInput restingPositionLimitSwitch;
    private final DigitalInput scoringPositionLimitSwitch;
    private final PWMVictorSPX m_motor = new PWMVictorSPX(ArmConstants.kMotorPort);
    private final Encoder m_encoder =
        new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]); 
    private final ArmFeedforward m_feedforward =
        new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kCosVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  
    /** Create a new ArmSubsystem. */
    public Arm(int restingPosChannel, int scoringPosChannel) {
        super(
            new ProfiledPIDController(
                ArmConstants.kP,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    ArmConstants.kMaxVelocityRadPerSecond,
                    ArmConstants.kMaxAccelerationRadPerSecSquared)),
            0);
        m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
        restingPositionLimitSwitch = new DigitalInput(restingPosChannel);
        scoringPositionLimitSwitch = new DigitalInput(scoringPosChannel);
      // Start arm at rest in neutral position
      setGoal(ArmConstants.kArmOffsetRads);
    }
  
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
      // Calculate the feedforward from the sepoint
      double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
      // Add the feedforward to the PID output to get the motor output
      final double batteryVoltage = RobotController.getBatteryVoltage();
      rawMotor((output + feedforward) / batteryVoltage);
    }
  
    @Override
    public double getMeasurement() {
      return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
    }

    public void rawMotor(double speed) {
        m_motor.set(speed);
    }

    public boolean getEnabledState() {
        return m_enabled;
    }

    public boolean getLimitSwitchResting() {
        return restingPositionLimitSwitch.get();
    }

    public void resetEncoder() {
        m_encoder.reset();
    }
}