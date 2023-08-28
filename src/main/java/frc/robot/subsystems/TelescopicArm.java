// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TelescopicArmConstants;

public class TelescopicArm extends SubsystemBase {
  /** Creates a new telscopicArm. */
  private WPI_TalonFX telesMotor;
  private DigitalInput homeSwitch;

  public TelescopicArm() {
    telesMotor = new WPI_TalonFX(TelescopicArmConstants.teleMotorID);
    homeSwitch = new DigitalInput(TelescopicArmConstants.homeSwitchID);

    setupTelesMotor();
  }
  
  public void setupTelesMotor() {
    telesMotor.setInverted(true);
    telesMotor.configSupplyCurrentLimit(Constants.ARM_MOTOR_SUPPLY_CONFIG);

    telesMotor.setNeutralMode(NeutralMode.Brake);

    telesMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TelescopicArmConstants.kPIDLoopIdx,
        TelescopicArmConstants.kTimeoutMs);
    telesMotor.configNeutralDeadband(0.001, TelescopicArmConstants.kTimeoutMs);

    telesMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TelescopicArmConstants.kTimeoutMs);
    telesMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TelescopicArmConstants.kTimeoutMs);

    telesMotor.configNominalOutputForward(0, TelescopicArmConstants.kTimeoutMs);
    telesMotor.configNominalOutputReverse(0, TelescopicArmConstants.kTimeoutMs);
    telesMotor.configPeakOutputForward(1, TelescopicArmConstants.kTimeoutMs);
    telesMotor.configPeakOutputReverse(-1, TelescopicArmConstants.kTimeoutMs);

    telesMotor.selectProfileSlot(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.kPIDLoopIdx);
    telesMotor.config_kF(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.TelesKF, TelescopicArmConstants.kTimeoutMs);
    telesMotor.config_kP(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.TelesKP, TelescopicArmConstants.kTimeoutMs);
    telesMotor.config_kI(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.TelesKI, TelescopicArmConstants.kTimeoutMs);
    telesMotor.config_kD(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.TelesKD, TelescopicArmConstants.kTimeoutMs);

    telesMotor.configMotionCruiseVelocity(15000, TelescopicArmConstants.kTimeoutMs);
    telesMotor.configMotionAcceleration(6000, TelescopicArmConstants.kTimeoutMs);

    telesMotor.setSelectedSensorPosition(0, TelescopicArmConstants.kPIDLoopIdx, TelescopicArmConstants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Big Arm Position", telesMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Home Switch value", homeSwitch.get());
  }

  public Command putTelesInSetpoint(double setPoint) {
    return runOnce(() -> telesMotor.set(TalonFXControlMode.MotionMagic, setPoint));
  }

  public void zeroTeles() {
    telesMotor.setNeutralMode(NeutralMode.Coast);
    while (!homeSwitch.get()) {}
    telesMotor.setSelectedSensorPosition(0);
    telesMotor.setNeutralMode(NeutralMode.Brake);
  }
  
  public Command setMotorSpeed(Double speed) {
    return this.run(() -> {

      telesMotor.set(speed);
    });
  }
}
