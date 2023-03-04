package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private boolean shouldGripCone;
  private Solenoid gripperOpenSolenoid;
  private Solenoid gripperCloseSolenoid;
  private DigitalInput isGripperOpen;
  
  private AddressableLED m_led = new AddressableLED(0);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(55);
  private boolean fakeIsGripperOpen;

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
    
    m_led.setLength(m_ledBuffer.getLength());
    
    fakeIsGripperOpen = false;

    turnOnLed();
  }

  public boolean getShouldGripCone() {
    return this.shouldGripCone;
  }

  public void toggleShouldWeGripACone() {
    this.shouldGripCone = !shouldGripCone;
  }

  public boolean isGripperOpen(){
    return fakeIsGripperOpen;
  }

  public Command toggleGripper(){
    return new ConditionalCommand(closeGripper(), openGripper(), this::isGripperOpen);
  }

  public void setShouldGripCone(boolean shouldGripCone) {
    this.shouldGripCone = shouldGripCone;
  }

  public Command openGripper(){
    return runOnce(()->{
      fakeIsGripperOpen = true;
        gripperCloseSolenoid.set(false);
        gripperOpenSolenoid.set(true);
    });
  }

  public Command closeGripper(){
    return runOnce(()->{
      fakeIsGripperOpen = false;
        gripperCloseSolenoid.set(true);
        gripperOpenSolenoid.set(false);
      // else{
      //   gripperCloseSolenoid.set(false);
      //   gripperOpenSolenoid.set(false);
      //}
    });
  }

  public Command changeWhatWeGrip() {
    return runOnce(() -> {
      toggleShouldWeGripACone();

      if(shouldGripCone){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) 
          m_ledBuffer.setRGB(i, 255, 100, 0);
        }
      else{
        for (var i = 0; i < m_ledBuffer.getLength(); i++)
          m_ledBuffer.setRGB(i,180,31,235);
        }
      
     
      m_led.setData(m_ledBuffer);
      m_led.start();
    });
  }

public void turnOffLed() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
      m_ledBuffer.setRGB(i,0,0,0);
    
    m_led.setData(m_ledBuffer);
    m_led.start();
}

public void turnOnLed() {
    if(shouldGripCone){
      for (var i = 0; i < m_ledBuffer.getLength(); i++) 
        m_ledBuffer.setRGB(i, 255, 100, 0);
      }
    else{
      for (var i = 0; i < m_ledBuffer.getLength(); i++)
        m_ledBuffer.setRGB(i,180,31,235);
      }
    
   
    m_led.setData(m_ledBuffer);
    m_led.start();
}

public boolean getFakeIsGripperOpen(){
  return fakeIsGripperOpen;
}

}
