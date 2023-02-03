package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BigArmConstants;

public class BigArm extends SubsystemBase {
  private WPI_TalonSRX  BigArmMotor;
  private PIDController BigArmPID;
  private ArmFeedforward BigArmMotorFeedforward;

  public BigArm() {
    BigArmPID = BigArmConstants.BigArmPID;
    BigArmMotorFeedforward = BigArmConstants.BigArmFeedforward;

    BigArmMotor = new WPI_TalonSRX(BigArmConstants.BigArmMotorPort);
  }

  public void putBigArmInPose( double setpoint){
    //TODO: adjust shit to the gear ratio
    BigArmMotor.setVoltage(BigArmPID.calculate(BigArmMotor.getSelectedSensorPosition() , setpoint)  + BigArmMotorFeedforward.calculate(setpoint,BigArmConstants.feedForwardVelocity));
  }

  public Command TurnToSetPoint(double setPoint){
    return run(()->{
      putBigArmInPose(setPoint);
    } );
  }
}
