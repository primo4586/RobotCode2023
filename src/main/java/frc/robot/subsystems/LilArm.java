package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LilArmConstants;

public class LilArm extends SubsystemBase {
  private WPI_TalonSRX leftLilArmMotor;
  private WPI_TalonSRX rightLilArmMotor;

  private Solenoid lilArmSolenoid;

  private PIDController lilArmPID;
  private ArmFeedforward lilArmFeedforward;

  /** Creates a new LilArm. */
  public LilArm() {
    lilArmPID = LilArmConstants.lilArmPID;
    lilArmFeedforward = LilArmConstants.lilArmFeedforward;

    leftLilArmMotor = new WPI_TalonSRX(Constants.LilArmConstants.leftLilMotorID);
    rightLilArmMotor = new WPI_TalonSRX(Constants.LilArmConstants.rightLilMotorID);
    lilArmSolenoid = new Solenoid(LilArmConstants.PCMID, PneumaticsModuleType.CTREPCM,
        LilArmConstants.lilArmSolenoidID);

    rightLilArmMotor.setInverted(true);

    leftLilArmMotor.setSelectedSensorPosition(Preferences.getDouble(LilArmConstants.armPostionKey, LilArmConstants.defultArmPose));
  }

  public boolean isSolenoidOpen() {
    return lilArmSolenoid.get();
  }

  // TODO: adjust shit to the gear ratio
  public void putLilArmInState(TrapezoidProfile.State setpointState) {
    leftLilArmMotor.setVoltage(lilArmPID.calculate(getCurrentArmAngle(), setpointState.position)
        + lilArmFeedforward.calculate(setpointState.position, setpointState.velocity));
    rightLilArmMotor.setVoltage(lilArmPID.calculate(getCurrentArmAngle(), setpointState.position)
        + lilArmFeedforward.calculate(setpointState.position, setpointState.velocity));
  }

  /**
   * Turns the arm to the given setpoint.
   * 
   * @param setPoint target angle, given in degrees.
   * @return A command that turn the angle to the setpoint.
   */
  public Command turnToSetPoint(double setPoint) {
    TrapezoidProfile profile = new TrapezoidProfile(LilArmConstants.lilArmProfileConstraints,
        new TrapezoidProfile.State(setPoint, 0));
    Timer timer = new Timer();
    return run(() -> {
      var state = profile.calculate(timer.get());

      putLilArmInState(state);
    })
        .beforeStarting(timer::start, this)
        .until(() -> Math.abs(this.getCurrentArmAngle() - setPoint) <= LilArmConstants.angleTolarance
            || profile.isFinished(timer.get()))
        .finallyDo((interrupted) -> timer.stop());
  }

  // TODO: Setup conversions according to the encoder's CPR
  public double getCurrentArmAngle() {
    return leftLilArmMotor.getSelectedSensorPosition();
  }

  public Command openLilArmSolenoid() {
    return runOnce(() -> {
      lilArmSolenoid.set(true);
    });
  }

  public Command closeLilArmSolenoid() {
    return runOnce(() -> {
      lilArmSolenoid.set(false);
    });
  }
}
