package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    leftLilArmMotor = new WPI_TalonSRX(Constants.LilArmConstants.leftLilMotorID);
    rightLilArmMotor = new WPI_TalonSRX(Constants.LilArmConstants.rightLilMotorID);
    lilArmSolenoid = new Solenoid(LilArmConstants.PCMID,PneumaticsModuleType.CTREPCM,LilArmConstants.lilArmSolenoidID);

    rightLilArmMotor.setInverted(true);

  }

  public void toggleSolenoidState() {
    lilArmSolenoid.toggle();
  }

  public boolean isSolenoidOpen(){
    return lilArmSolenoid.get();
  }

  public void putLilArmInPose( Double setpoint) {
    leftLilArmMotor.setVoltage(lilArmPID.calculate(leftLilArmMotor.getSelectedSensorPosition(),setpoint) + lilArmFeedforward.calculate(setpoint, Constants.LilArmConstants.lilArmFeedForwardVelocity));
    rightLilArmMotor.setVoltage(lilArmPID.calculate(leftLilArmMotor.getSelectedSensorPosition(),setpoint) + lilArmFeedforward.calculate(setpoint, Constants.LilArmConstants.lilArmFeedForwardVelocity));
  } 
  
//TODO: adjust shit to the gear ratio
  public Command turnToSetPoint(Double setPoint){
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
