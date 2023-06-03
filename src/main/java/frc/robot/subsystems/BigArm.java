package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BigArmConstants;

public class BigArm extends SubsystemBase {
  private CANSparkMax bigArmMotor;
  private WPI_TalonSRX bigArmEncoder;
  private PIDController bigArmPID;
  private DigitalInput honeSwitch;

  public BigArm() {
    bigArmPID = BigArmConstants.bigArmPID;

    // TODO: I assumed we're using the NEO 550s which are brushless, change motor type if not. 
    // TODO: were using neo v1.1 not 550 nut it doesn't matter
    bigArmMotor = new CANSparkMax(BigArmConstants.bigArmMotorID, MotorType.kBrushless); 
    bigArmEncoder = new WPI_TalonSRX(BigArmConstants.bigArmEncoderID);
    
    bigArmEncoder.setSelectedSensorPosition(BigArmConstants.intakeSetPoint);
    bigArmMotor.setSmartCurrentLimit(Constants.ARM_STALL_CURRENT_LIMIT, Constants.ARM_FREE_CURRENT_LIMIT);

    honeSwitch = new DigitalInput(BigArmConstants.honeSwitchID);
    bigArmMotor.setInverted(true); // TODO: Double-check inverts as necessary 
    var errorCode = bigArmMotor.burnFlash();
    System.out.println("BigArm Error Code:" + errorCode);
  }

  public void putBigArmInPlace(double setPoint){
    SmartDashboard.putNumber("Big Arm PID Output",bigArmPID.calculate(getCurrentArmPosition(), setPoint));
    bigArmMotor.setVoltage(bigArmPID.calculate(getCurrentArmPosition(), setPoint));
  }

  public void zeroEncoderForIntake(){
    this.bigArmEncoder.setSelectedSensorPosition(BigArmConstants.intakeSetPoint);
  }

  public Command TurnBigArmToSetpoint(double setPoint){
    SmartDashboard.putNumber("Big Arm Setpoint", setPoint);
    return run(()->{
      putBigArmInPlace(setPoint);
    })
    .until(() -> Math.abs(this.getCurrentArmPosition() - setPoint) <= BigArmConstants.ticksTolerance);
  }

  public double getCurrentArmPosition() {
    return bigArmEncoder.getSelectedSensorPosition();
  }

  public Command setMotorSpeed(DoubleSupplier supplier) {
    return this.run(() -> {

      bigArmMotor.set(supplier.getAsDouble());
    });
  }



  public void setPreference(){
    Preferences.setDouble(BigArmConstants.bigArmPreferencesKey, getCurrentArmPosition());
  }

  public boolean getHoneSwitch(){
    return !honeSwitch.get();
  }

  public Command Hone(){
    return run(()->{
      bigArmMotor.set(BigArmConstants.honeSpeed);
    })
    .until(()->getHoneSwitch())
    .andThen(()->{bigArmMotor.set(0.0);
      bigArmEncoder.setSelectedSensorPosition(BigArmConstants.honeSetPoint);
    });
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Big Arm Position", bigArmEncoder.getSelectedSensorPosition());
      SmartDashboard.putBoolean("Hone Switch value", getHoneSwitch());
  }

  public Command zeroEncoder() {
    return runOnce(() -> 
      bigArmEncoder.setSelectedSensorPosition(0)
    );
  }
}
