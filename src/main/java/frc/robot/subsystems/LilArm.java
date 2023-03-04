package frc.robot.subsystems;

import java.util.function.DoubleSupplier;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
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


  /** Creates a new LilArm. */
  public LilArm() {
    lilArmPID = LilArmConstants.lilArmPID;

    lilArmEncoder = new WPI_TalonSRX(LilArmConstants.lilArmEncoderID);
    lilArmMotor = new WPI_TalonSRX(LilArmConstants.lilArmMotorID);
    lilArmSolenoid = new Solenoid(LilArmConstants.PCMID, PneumaticsModuleType.CTREPCM,
        LilArmConstants.lilArmSolenoidID);
    lilArmMotor.configSupplyCurrentLimit(Constants.ARM_MOTOR_SUPPLY_CONFIG);    
 
    lilArmEncoder.setSelectedSensorPosition(LilArmConstants.intakeSetPoint);
    solenoidOpenSensor = new DigitalInput(0);
  }

  public boolean isSolenoidOpen() {
    return lilArmSolenoid.get();
  }

  public void zeroEncoderForIntake(){
    this.lilArmEncoder.setSelectedSensorPosition(LilArmConstants.intakeSetPoint);
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

  public Command speedByTime(double speed, double time) {
    Timer timer = new Timer();
    return setMotorSpeed(() -> speed).beforeStarting(() -> timer.start()).until(() -> timer.hasElapsed(time));
  }


  public void setPreference(){
    //Preferences.setDouble(LilArmConstants.lilArmPreferencesKey, getCurrentArmAngle());
    Preferences.setDouble(LilArmConstants.lilArmPreferencesKey, LilArmConstants.resetPoint);
  }

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
