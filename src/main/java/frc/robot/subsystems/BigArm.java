package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BigConstants;

public class BigArm extends SubsystemBase {
  private TalonFX bigArmMotor;
  private DigitalInput homeSwitch;

  private MotionMagicVoltage motionMagicVoltage;

  public BigArm() {
    bigArmMotor = new TalonFX(BigConstants.bigArmMotorID);
    homeSwitch = new DigitalInput(BigConstants.homeSwitchID);
    motionMagicVoltage = new MotionMagicVoltage(0, true, BigConstants.bigArmKV, 0, false);
    setupBigArmMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Big Arm Position", bigArmMotor.getRotorPosition().getValue());
    SmartDashboard.putBoolean("Home Switch value", homeSwitch.get());
  }

  public void setupBigArmMotor() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kP = BigConstants.bigArmKP;
    motorConfig.Slot0.kI = BigConstants.bigArmKI;
    motorConfig.Slot0.kD = BigConstants.bigArmKD;
    motorConfig.Slot0.kV = BigConstants.bigArmKV;
    motorConfig.Slot0.kS = BigConstants.bigArmKS;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = BigConstants.maxSpeed;
    motorConfig.MotionMagic.MotionMagicAcceleration = BigConstants.maxAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = BigConstants.maxJerk;
    motionMagicVoltage.Slot = 0;
    
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.armEnableCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = Constants.armContinuousCurrentLimit;
    motorConfig.CurrentLimits.SupplyTimeThreshold = Constants.armPeakCurrentDuration;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.armPeakCurrentLimit;
    
    bigArmMotor.getConfigurator().apply(motorConfig);

    bigArmMotor.setRotorPosition(0);
  }

  public void zeroEncoderForMiddleOfBot() {
    bigArmMotor.setRotorPosition(BigConstants.intakeSetPoint);
  }

  public Command TurnBigArmToSetpoint(double setPoint) {
    SmartDashboard.putNumber("Big Arm Setpoint", setPoint);
    return runOnce(() -> {
      bigArmMotor.setControl(motionMagicVoltage.withPosition(0));
    });
  }

  public double getCurrentArmPosition() {
    return bigArmMotor.getRotorPosition().getValue();
  }

  public Command setMotorSpeed(DoubleSupplier speed) {
    return this.run(() -> {
      bigArmMotor.set(speed.getAsDouble());
    });
  }

  public Command Home() {
    return run(() -> {
      bigArmMotor.set(BigConstants.homeSpeed);
    })
        .until(() -> homeSwitch.get())
        .andThen(() -> {
          bigArmMotor.set(0.0);
          bigArmMotor.setRotorPosition(BigConstants.homeSetPoint);
        });
  }

  public Command zeroEncoder() {
    return runOnce(() -> bigArmMotor.setRotorPosition(0));
  }
}
