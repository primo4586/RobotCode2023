// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LilArmConstants;


public class LilArm extends SubsystemBase {
  private WPI_TalonSRX leftLilArmMotor;
  private WPI_TalonSRX rightLilArmMotor;

  private Solenoid lilArmSolenoid;

  private PIDController lilArmPID;
  private ArmFeedforward lilArmFeedforward;

  /** Creates a new LilArm. */
  public LilArm() {
    lilArmPID =LilArmConstants.lilArmPID;
    lilArmFeedforward =LilArmConstants.lilArmFeedforward;

    leftLilArmMotor = new WPI_TalonSRX(LilArmConstants.leftLilMotorID);
    rightLilArmMotor = new WPI_TalonSRX(LilArmConstants.rightLilMotorID);
    lilArmSolenoid = new Solenoid(LilArmConstants.PCMID,PneumaticsModuleType.CTREPCM,LilArmConstants.lilArmSolenoidID);

    rightLilArmMotor.setInverted(true);
  }

  public void toggleSolenoidState() {
    lilArmSolenoid.toggle();
  }

  public BooleanSupplier isSolenoidOpen(){
    return ()-> lilArmSolenoid.get();
  }

  public void putLilArmInPose(double setpoint) {
    leftLilArmMotor.setVoltage(lilArmPID.calculate(leftLilArmMotor.getSelectedSensorPosition(),setpoint) + lilArmFeedforward.calculate(setpoint,LilArmConstants.lilArmFeedForwardVelocity));
    rightLilArmMotor.setVoltage(lilArmPID.calculate(leftLilArmMotor.getSelectedSensorPosition(),setpoint) + lilArmFeedforward.calculate(setpoint,LilArmConstants.lilArmFeedForwardVelocity));
  } 
  
//TODO: adjust shit to the gear ratio
  public Command turnToSetPoint(double setPoint){
    return run(()->{
      putLilArmInPose(setPoint);
    } ).until(()-> leftLilArmMotor.getSelectedSensorPosition() == setPoint);
  }

  public Command toggleLilArmSolenoid() {
    return runOnce(()->{
      toggleSolenoidState();
    } );
  }
}
