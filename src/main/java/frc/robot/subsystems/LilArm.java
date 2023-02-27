package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LilArmConstants;

public class LilArm extends SubsystemBase {
  private WPI_TalonSRX lilArmMotor;
  private WPI_TalonSRX lilArmEncoder;

  private Solenoid lilArmSolenoid;
  private DigitalInput solenoidOpenSensor;

  private PIDController lilArmPID;
  private ArmFeedforward lilArmFeedforward;


  /** Creates a new LilArm. */
  public LilArm() {
    lilArmPID = LilArmConstants.lilArmPID;
    lilArmFeedforward = LilArmConstants.lilArmFeedforward;

    lilArmEncoder = new WPI_TalonSRX(LilArmConstants.lilArmEncoderID);
    lilArmMotor = new WPI_TalonSRX(LilArmConstants.lilArmMotorID);
    lilArmSolenoid = new Solenoid(LilArmConstants.PCMID, PneumaticsModuleType.CTREPCM,
        LilArmConstants.lilArmSolenoidID);
 
    lilArmEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    // lilArmEncoder.setSelectedSensorPosition(Preferences.getDouble(LilArmConstants.lilArmPreferencesKey, 0));
    // 
    solenoidOpenSensor = new DigitalInput(0);
  }

  public boolean isSolenoidOpen() {
    return lilArmSolenoid.get();
  }

  public Command setMotorSpeed(DoubleSupplier supplier) {
    return this.run(() -> {

      lilArmMotor.set(supplier.getAsDouble() * 0.5);
    });
  }

  public void putArmInPlace(double setpoint) {
    SmartDashboard.putNumber("LilArm PID Output", lilArmPID.calculate(getCurrentArmAngle(), setpoint));
    lilArmMotor.setVoltage(lilArmPID.calculate(getCurrentArmAngle(), setpoint));
  }

  public Command TurnLilArmToSetpoint(double setpoint) {
    SmartDashboard.putNumber("LilArm Setpoint", setpoint);
    return run(() -> {
        putArmInPlace(setpoint);
    })
    .until(() -> (Math.abs(getCurrentArmAngle() - setpoint) <= LilArmConstants.ticksTolerance));
  }

  public void setPreference(){
    //Preferences.setDouble(LilArmConstants.lilArmPreferencesKey, getCurrentArmAngle());
    Preferences.setDouble(LilArmConstants.lilArmPreferencesKey, LilArmConstants.resetPoint);
  }

  // TODO: Setup conversions according to the encoder's CPR
  public double getCurrentArmAngle() {
    return lilArmEncoder.getSelectedSensorPosition();
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

  public Command zeroLilArm() {
    return runOnce(() -> {
      lilArmEncoder.setSelectedSensorPosition(-30);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LilArm Position", lilArmEncoder.getSelectedSensorPosition());
  }
}
