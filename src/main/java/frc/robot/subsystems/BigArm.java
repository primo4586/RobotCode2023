package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BigArmConstants;

public class BigArm extends SubsystemBase {
  private WPI_TalonSRX bigArmMotor;
  private PIDController bigArmPID;
  private ArmFeedforward bigArmMotorFeedforward;
  private DigitalInput honeSwitch;

  public BigArm() {
    bigArmPID = BigArmConstants.bigArmPID;
    bigArmMotorFeedforward = BigArmConstants.bigArmFeedforward;

    bigArmMotor = new WPI_TalonSRX(BigArmConstants.bigArmMotorPort);

    honeSwitch = new DigitalInput(BigArmConstants.honeSwitchID);
  }

  public void putBigArmInState(TrapezoidProfile.State setpointState) {
    // TODO: adjust shit to the gear ratio
    bigArmMotor.setVoltage(bigArmPID.calculate(getCurrentArmAngle(), setpointState.position)
        + bigArmMotorFeedforward.calculate(setpointState.position, setpointState.velocity));
  }

  /**
   * Turns the arm to the given setpoint.
   * 
   * @param setPoint target angle, given in degrees.
   * @return A command that turn the angle to the setpoint.
   */
  public Command turnToSetPoint(double setPoint) {
    TrapezoidProfile profile = new TrapezoidProfile(BigArmConstants.bigArmProfileConstraints,
        new TrapezoidProfile.State(setPoint, 0));
    Timer timer = new Timer();
    return run(() -> {
      var state = profile.calculate(timer.get());

      putBigArmInState(state);
    })
    .beforeStarting(timer::start, this)
    .until(() -> Math.abs(this.getCurrentArmAngle() - setPoint) <= BigArmConstants.angleTolarance || profile.isFinished(timer.get()))
    .finallyDo((interrupted) -> timer.stop());
  }

  // TODO: Setup conversions according to the encoder's CPR
  public double getCurrentArmAngle() {
    return bigArmMotor.getSelectedSensorPosition();
  }

  public Command Hone(){
    return run(()->{
      bigArmMotor.set(BigArmConstants.honeSpeed);
    })
    .until(()->honeSwitch.get())
    .andThen(()->{bigArmMotor.set(0.0);
      bigArmMotor.setSelectedSensorPosition(BigArmConstants.honeSetPoint);
    });
  }
}
