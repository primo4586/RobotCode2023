package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BigArmConstants;

public class BigArm extends SubsystemBase {
  private WPI_TalonSRX bigArmMotor;
  private WPI_TalonSRX bigArmEncoder;
  private PIDController bigArmPID;
  private DigitalInput honeSwitch;

  public BigArm() {
    bigArmPID = BigArmConstants.bigArmPID;

    bigArmMotor = new WPI_TalonSRX(BigArmConstants.bigArmMotorPort);
    bigArmEncoder = new WPI_TalonSRX(BigArmConstants.bigArmEncoderID);
    
    bigArmEncoder.setSelectedSensorPosition(BigArmConstants.intakeSetPoint);
    bigArmMotor.configSupplyCurrentLimit(Constants.ARM_MOTOR_SUPPLY_CONFIG);

    bigArmMotor.setInverted(true);
    bigArmEncoder.setInverted(true);

    honeSwitch = new DigitalInput(BigArmConstants.honeSwitchID);
  }

  public void putBigArmInPlace(double setPoint){
    SmartDashboard.putNumber("Big Arm PID Output",-bigArmPID.calculate(getCurrentArmAngle(), setPoint));
    bigArmMotor.setVoltage(-bigArmPID.calculate(getCurrentArmAngle(), setPoint));
  }

  public void zeroEncoderForIntake(){
    this.bigArmEncoder.setSelectedSensorPosition(BigArmConstants.intakeSetPoint);
  }

  public Command TurnBigArmToSetpoint(double setPoint){
    SmartDashboard.putNumber("Big Arm Setpoint", setPoint);
    return run(()->{
      putBigArmInPlace(setPoint);
    })
    .until(() -> Math.abs(this.getCurrentArmAngle() - setPoint) <= BigArmConstants.ticksTolerance);
  }

  public double getCurrentArmAngle() {
    return bigArmEncoder.getSelectedSensorPosition();
  }

  public Command setMotorSpeed(DoubleSupplier supplier) {
    return this.run(() -> {

      bigArmMotor.set(supplier.getAsDouble() * 0.5);
    });
  }

  public void setPreference(){
    Preferences.setDouble(BigArmConstants.bigArmPreferencesKey, getCurrentArmAngle());
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
  }

  public Command zeroEncoder() {
    return runOnce(() -> 
      bigArmEncoder.setSelectedSensorPosition(0)
    );
  }
}
