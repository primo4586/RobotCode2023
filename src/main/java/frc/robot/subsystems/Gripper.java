package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private WPI_TalonSRX  gripperMotor;
  private PIDController gripperPID;
  private SimpleMotorFeedforward gripperMotorFeedforward;
  private boolean doWeGripACone;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperPID = GripperConstants.gripperPID;
    gripperMotorFeedforward = GripperConstants.gripperFeedforward;

    gripperMotor = new WPI_TalonSRX(GripperConstants.gripperMotorPort);

    doWeGripACone = true;
  }

  public boolean getDoWeGripACone(){
    return this.doWeGripACone;
  }

  public void changeDoWeGripACone(){
    this.doWeGripACone = !doWeGripACone;
  }

  public void putGripperInPose( double setpoint){
    gripperMotor.setVoltage(gripperPID.calculate(gripperMotor.getSelectedSensorPosition() , setpoint)  + gripperMotorFeedforward.calculate(setpoint));
  }

  public Command TurnToSetPoint(double setPoint){
    return run(()->{
      putGripperInPose(setPoint);
    });
  }

  public Command ChangeWhatWeGrip(){
    return runOnce(()->{
      changeDoWeGripACone();
    });
    
  }
}
