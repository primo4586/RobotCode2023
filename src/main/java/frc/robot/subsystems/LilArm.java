package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LilArmConstants;

public class LilArm extends SubsystemBase {
  private CANSparkMax lilArmMotor;
  private WPI_TalonSRX lilArmEncoder;

  private Solenoid lilArmSolenoid;
  private PIDController lilArmPID;


  /** Creates a new LilArm. */
  public LilArm() {
    lilArmPID = LilArmConstants.lilArmPID;

    lilArmEncoder = new WPI_TalonSRX(LilArmConstants.lilArmEncoderID);
    // I assumed we're using the NEO 550s which are brushless, change motor type if not.
    // were using neo v1.1 not 550 nut it doesn't matter
    lilArmMotor = new CANSparkMax(LilArmConstants.lilArmMotorID, MotorType.kBrushless);
    lilArmSolenoid = new Solenoid(LilArmConstants.PCMID, PneumaticsModuleType.CTREPCM,
        LilArmConstants.lilArmSolenoidID);

    lilArmMotor.setInverted(false); 
    lilArmMotor.setSmartCurrentLimit(Constants.ARM_STALL_CURRENT_LIMIT, Constants.ARM_FREE_CURRENT_LIMIT);

    lilArmEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    var errorCode = lilArmMotor.burnFlash();
    System.out.println("LilArm Error Code: " + errorCode);
  }

  public boolean isSolenoidOpen() {
    return lilArmSolenoid.get();
  }

  public void zeroEncoderForIntake() {
    this.lilArmEncoder.setSelectedSensorPosition(LilArmConstants.intakeSetPoint);
  }

  public void zeroEncoderForAuto() {
    this.lilArmEncoder.setSelectedSensorPosition(LilArmConstants.autoStartPoint);
  }

  public Command setMotorSpeed(DoubleSupplier supplier) {
    return this.run(() -> {

      lilArmMotor.set(supplier.getAsDouble());
    });
  }

  public void putArmInPlace(double setpoint) {
    SmartDashboard.putNumber("LilArm PID Output", lilArmPID.calculate(getCurrentArmPosition(), setpoint));
    if(lilArmPID.calculate(getCurrentArmPosition(),setpoint)>12){
      lilArmMotor.set(1);
    }
    else if(lilArmPID.calculate(getCurrentArmPosition(),setpoint)<-12){
      lilArmMotor.set(-1);
    }
    else{
    lilArmMotor.set(lilArmPID.calculate(getCurrentArmPosition(),setpoint)/12);
    }
    //lilArmMotor.setVoltage(lilArmPID.calculate(getCurrentArmPosition(), setpoint));
  }

  public void putArmInPlaceWithCustomPID(double setpoint, PIDController pid) {
    // SmartDashboard.putNumber("LilArm PID Output",
    // pid.calculate(getCurrentArmPosition(), setpoint));
    lilArmMotor.setVoltage(pid.calculate(getCurrentArmPosition(), setpoint));
  }

  public Command TurnLilArmToSetpoint(double setpoint) {
    SmartDashboard.putNumber("LilArm Setpoint", setpoint);
    return run(() -> {
      putArmInPlace(setpoint);
    })
        .until(() -> (Math.abs(getCurrentArmPosition() - setpoint) <= LilArmConstants.ticksTolerance));
  }

  public Command TurnLilArmToSetpointOnlyForAuto(double setpoint) {
    // SmartDashboard.putNumber("LilArm Setpoint", setpoint);
    return run(() -> {
      putArmInPlace(setpoint);
    })
        .until(() -> (Math.abs(getCurrentArmPosition() - setpoint) <= LilArmConstants.ticksTolerance));
  }

  public Command TurnLilArmToSetpointWithCustomPID(double setpoint, PIDController pid) {
    // SmartDashboard.putNumber("LilArm Setpoint", setpoint);
    return run(() -> {
      putArmInPlaceWithCustomPID(setpoint, pid);
    })
        .until(() -> (Math.abs(getCurrentArmPosition() - setpoint) <= LilArmConstants.ticksTolerance));
  }

  public Command speedByTime(double speed, double time) {
    Timer timer = new Timer();
    return setMotorSpeed(() -> speed).beforeStarting(() -> timer.start()).until(() -> timer.hasElapsed(time));
  }

  public void setPreference() {
    Preferences.setDouble(LilArmConstants.lilArmPreferencesKey, LilArmConstants.resetPoint);
  }

  public double getCurrentArmPosition() {
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
      lilArmEncoder.setSelectedSensorPosition(0);
    });
  }

  public double getSpeed(){
    return lilArmMotor.get();
  }

  public void setEncoder(int encoderSpot){
    lilArmEncoder.setSelectedSensorPosition(encoderSpot);
  }

  public int getEncoder(){
    return lilArmEncoder.getSensorCollection().getQuadraturePosition();
  }


  @Override
  public void simulationPeriodic() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LilArm Position", lilArmEncoder.getSelectedSensorPosition());
  }
}
