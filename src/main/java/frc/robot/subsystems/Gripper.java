package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private WPI_TalonSRX  gripperMotor;
  private PIDController gripperPID;
  private SimpleMotorFeedforward gripperMotorFeedforward;
  private boolean shouldGripCone;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperPID = GripperConstants.gripperPID;
    gripperMotorFeedforward = GripperConstants.gripperFeedforward;

    gripperMotor = new WPI_TalonSRX(GripperConstants.gripperMotorPort);

    shouldGripCone = true;
  }

  public boolean getShouldGripCone(){
    return this.shouldGripCone;
  }

  public void toggleShouldWeGripACone(){
    this.shouldGripCone = !shouldGripCone;
  }

  private void putGripperInPose( double setpoint){
    gripperMotor.setVoltage(gripperPID.calculate(gripperMotor.getSelectedSensorPosition() , setpoint)  + gripperMotorFeedforward.calculate(setpoint));
  }

  //TODO: adjust shit to the gear ratio
  public Command turnToSetPoint(double setPoint){
    return run(()->{
      putGripperInPose(setPoint);
    }).until(() -> Math.abs(gripperMotor.getSelectedSensorPosition()-setPoint) <= GripperConstants.grippingTolarance);
  }

  public Command changeWhatWeGrip(){
    return runOnce(()->{
      toggleShouldWeGripACone();
    });
    
  }

  public Command gripItem(){
    return runOnce(()->{
      if(shouldGripCone)
        turnToSetPoint(GripperConstants.coneGrabingSetPoint);
      else
        turnToSetPoint(GripperConstants.cubeGrabingSetPoint);
    });
  }
}
