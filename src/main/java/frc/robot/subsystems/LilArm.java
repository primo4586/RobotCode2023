// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.OpticEncoder;
import frc.robot.Constants;
import frc.robot.Constants.LilArmConstants;


public class LilArm extends SubsystemBase {
  private WPI_TalonSRX leftLilArmMotor;
  private WPI_TalonSRX rightLilArmMotor;

  private Solenoid lilArmSolenoid;

  private PIDController lilArmPID;
  private ArmFeedforward lilArmFeedforward;

  private OpticEncoder lilArmEncoder;

  /** Creates a new LilArm. */
  public LilArm() {
    lilArmPID = Constants.LilArmConstants.lilArmPID;
    lilArmFeedforward = Constants.LilArmConstants.lilArmFeedforward;

    leftLilArmMotor = new WPI_TalonSRX(Constants.LilArmConstants.leftLilMotorID);
    rightLilArmMotor = new WPI_TalonSRX(Constants.LilArmConstants.rightLilMotorID);
    lilArmSolenoid = new Solenoid(Constants.LilArmConstants.PHID,PneumaticsModuleType.REVPH,Constants.LilArmConstants.lilArmSolenoidID);

    rightLilArmMotor.setInverted(true);

    lilArmEncoder = new OpticEncoder(Constants.LilArmConstants.sensorID, ()-> leftLilArmMotor.get() > 0);

  }

  @Override
  public void periodic() {
      lilArmEncoder.update();
  }

  public void toggleSolenoidState() {
    lilArmSolenoid.toggle();
  }

  public void putLilArmInPose( int setpoint) {
    leftLilArmMotor.setVoltage(lilArmPID.calculate(lilArmEncoder.getPose(),setpoint) + lilArmFeedforward.calculate(setpoint, Constants.LilArmConstants.lilArmFeedForwardVelocity));
    rightLilArmMotor.setVoltage(lilArmPID.calculate(lilArmEncoder.getPose(),setpoint) + lilArmFeedforward.calculate(setpoint, Constants.LilArmConstants.lilArmFeedForwardVelocity));
  } 

  public Command turnToSetPoint(int setPoint){
    return run(()->{
      putLilArmInPose(setPoint);
    } );
  }

  public Command toggleLilArmSolenoid() {
    return runOnce(()->{
      toggleSolenoidState();
    } );
  }
}
