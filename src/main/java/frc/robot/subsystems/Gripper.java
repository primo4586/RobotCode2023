package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private boolean shouldGripCone;
  private Solenoid gripperOpenSolenoid;
  private Solenoid gripperCloseSolenoid;
  private DigitalInput isGripperOpen;

  /** Creates a new Gripper. */
  public Gripper() {

    shouldGripCone = true;

    gripperOpenSolenoid = new Solenoid(GripperConstants.PCMID, PneumaticsModuleType.CTREPCM, GripperConstants.solenoidOpenID);
    gripperCloseSolenoid = new Solenoid(GripperConstants.PCMID, PneumaticsModuleType.CTREPCM, GripperConstants.solenoidCloseID);
    if (shouldGripCone){
      gripperOpenSolenoid.set(false);
      gripperCloseSolenoid.set(true);
    }
    else{
      gripperOpenSolenoid.set(false);
      gripperCloseSolenoid.set(false);
    }

    isGripperOpen = new DigitalInput(GripperConstants.isGripperOpenID);
    
  }

  public boolean getShouldGripCone() {
    return this.shouldGripCone;
  }

  public void toggleShouldWeGripACone() {
    this.shouldGripCone = !shouldGripCone;
  }

  public boolean isGripperOpen(){
    return isGripperOpen.get();
  }

  public Command openGripper(){
    return runOnce(()->{
        gripperCloseSolenoid.set(false);
        gripperOpenSolenoid.set(true);
    });
  }

  public Command closeGripper(){
    return runOnce(()->{
      if (this.shouldGripCone){
        gripperCloseSolenoid.set(true);
        gripperOpenSolenoid.set(false);
      }
      else{
        gripperCloseSolenoid.set(false);
        gripperOpenSolenoid.set(false);
      }
    });
  }

  public Command changeWhatWeGrip() {
    return runOnce(() -> {
      toggleShouldWeGripACone();
    });
  }
}
