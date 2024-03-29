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
    telesMotor.configClosedLoopPeakOutput(0, 0.64, 30);

    telesMotor.selectProfileSlot(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.kPIDLoopIdx);
    telesMotor.config_kF(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.TelesKF, TelescopicArmConstants.kTimeoutMs);
    telesMotor.config_kP(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.TelesKP, TelescopicArmConstants.kTimeoutMs);
    telesMotor.config_kI(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.TelesKI, TelescopicArmConstants.kTimeoutMs);
    telesMotor.config_kD(TelescopicArmConstants.kSlotIdx, TelescopicArmConstants.TelesKD, TelescopicArmConstants.kTimeoutMs);

    telesMotor.configMotionCruiseVelocity(2000, TelescopicArmConstants.kTimeoutMs);
    telesMotor.configMotionAcceleration(6000, TelescopicArmConstants.kTimeoutMs);

    telesMotor.setSelectedSensorPosition(0, TelescopicArmConstants.kPIDLoopIdx, TelescopicArmConstants.kTimeoutMs);
    telesMotor.setNeutralMode(NeutralMode.Brake);

    telesMotor.configForwardSoftLimitThreshold(TelescopicArmConstants.softLimitForward);
    telesMotor.configReverseSoftLimitThreshold(TelescopicArmConstants.softLimitReverse);
    telesMotor.configForwardSoftLimitEnable(true);
    telesMotor.configReverseSoftLimitEnable(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Teles Position", telesMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Teles Switch value", homeSwitch.get());
    SmartDashboard.putNumber("teles error", telesMotor.getClosedLoopError());

    if(telesMotor.getSelectedSensorPosition()>32795){
      stop();
      telesMotor.set(0);
    }
  }

  public Command putTelesInSetpoint(double setPoint) {
    return run(() -> 
      telesMotor.set(TalonFXControlMode.Position, setPoint)
    ).until(()->Math.abs(telesMotor.getSelectedSensorPosition()-setPoint)<100).andThen(stop());
  }

  public void zeroTeles() {
    telesMotor.setNeutralMode(NeutralMode.Coast);
    while (homeSwitch.get()) {}
    telesMotor.setSelectedSensorPosition(0);
    telesMotor.setNeutralMode(NeutralMode.Brake);
  }

  public Command stop(){
    return this.runOnce(()->{
      telesMotor.set(0);
    });
  }
  
  public Command setMotorSpeed(Double speed) {
    return this.run(() -> {
      telesMotor.set(speed);
    });
  }

  public Command Home() {
    telesMotor.configForwardSoftLimitEnable(false);
    telesMotor.configReverseSoftLimitEnable(false);
    return run(() -> {
      telesMotor.set(-0.1);
    })
        .until(() -> !homeSwitch.get())
        .andThen(() -> {
          telesMotor.set(0.0);
          telesMotor.getSensorCollection().setIntegratedSensorPosition(0,30);
        });
  }
}
