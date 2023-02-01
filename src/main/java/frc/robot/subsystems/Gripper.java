package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private WPI_TalonSRX  gripperMotor;
  private Encoder gripperEncoder;
  private PIDController gripperPID;
  private SimpleMotorFeedforward gripperMotorFeedforward;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperPID = GripperConstants.gripperPID;
    gripperMotorFeedforward = GripperConstants.gripperFeedforward;

    gripperMotor = new WPI_TalonSRX(GripperConstants.gripperMotorPort);

    gripperEncoder = new Encoder(GripperConstants.gripperIncoderPortA, GripperConstants.gripperIncoderPortB);
    gripperEncoder.setDistancePerPulse(0.0);//TODO: add distancePerPulse
  }

  public void putGripperInPose(double pidOutpot , double setpoint){
    gripperMotor.setVoltage(pidOutpot + gripperMotorFeedforward.calculate(setpoint));
  }


  @Override
  public void periodic() {
    putGripperInPose(gripperPID.calculate(gripperEncoder.getRate()), gripperPID.getSetpoint());
  }
}
