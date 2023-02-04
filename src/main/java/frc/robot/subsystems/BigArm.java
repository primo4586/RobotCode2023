package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BigArmConstants;

public class BigArm extends SubsystemBase {
  private WPI_TalonSRX  bigArmMotor;
  private PIDController bigArmPID;
  private ArmFeedforward bigArmMotorFeedforward;

  public BigArm() {
    bigArmPID = BigArmConstants.bigArmPID;
    bigArmMotorFeedforward = BigArmConstants.bigArmFeedforward;

    bigArmMotor = new WPI_TalonSRX(BigArmConstants.bigArmMotorPort);
  }

  public void putbigArmInPose( double setpoint){
    //TODO: adjust shit to the gear ratio
    bigArmMotor.setVoltage(bigArmPID.calculate(bigArmMotor.getSelectedSensorPosition() , setpoint)  + bigArmMotorFeedforward.calculate(setpoint,BigArmConstants.feedForwardVelocity));
  }

  public Command turnToSetPoint(double setPoint){
    return run(()->{
      putbigArmInPose(setPoint);
    } ).until(() -> Math.abs(bigArmMotor.getSelectedSensorPosition()-setPoint) <= BigArmConstants.angleTolarance);
  }
}
