// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TelescopicArmConstants;

public class TelescopicArm extends SubsystemBase {
  /** Creates a new telscopicArm. */
  private TalonFX telesMotor;
  private DigitalInput homeSwitch;
  private MotionMagicVoltage motionMagicVoltage;

  public TelescopicArm() {
    telesMotor = new TalonFX(TelescopicArmConstants.teleMotorID);
    homeSwitch = new DigitalInput(TelescopicArmConstants.homeSwitchID);
    motionMagicVoltage = new MotionMagicVoltage(0, true, TelescopicArmConstants.TelesKV, 0, false);

    setupTelesMotor();
  }
  
  public void setupTelesMotor() {
    telesMotor.setInverted(true);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kP = TelescopicArmConstants.TelesKP;
    motorConfig.Slot0.kI = TelescopicArmConstants.TelesKI;
    motorConfig.Slot0.kD = TelescopicArmConstants.TelesKD;
    motorConfig.Slot0.kV = TelescopicArmConstants.TelesKV;
    motorConfig.Slot0.kS = TelescopicArmConstants.TelesKS;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = TelescopicArmConstants.maxSpeed;
    motorConfig.MotionMagic.MotionMagicAcceleration = TelescopicArmConstants.maxAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = TelescopicArmConstants.maxJerk;
    motionMagicVoltage.Slot = 0;

    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.armEnableCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = Constants.armContinuousCurrentLimit;
    motorConfig.CurrentLimits.SupplyTimeThreshold = Constants.armPeakCurrentDuration;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.armPeakCurrentLimit;

    telesMotor.getConfigurator().apply(motorConfig);

    telesMotor.setRotorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Big Arm Position", telesMotor.getRotorPosition().getValue());
    SmartDashboard.putBoolean("Home Switch value", homeSwitch.get());
  }

  public Command putTelesInSetpoint(double setPoint) {
    return runOnce(() -> telesMotor.setControl(motionMagicVoltage.withPosition(setPoint)));
  }

  public Command setMotorSpeed(DoubleSupplier speed) {
    return this.run(() -> {

      telesMotor.set(speed.getAsDouble());
    });
  }

  public void zeroTeles() {
    telesMotor.setControl(new CoastOut());
    while (!homeSwitch.get()) {}
    telesMotor.setRotorPosition(0);
    telesMotor.setControl(new NeutralOut());
  }
}
