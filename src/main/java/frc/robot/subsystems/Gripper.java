package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private boolean shouldGripCone;
  private Solenoid gripperSolenoid;

  /** Creates a new Gripper. */
  public Gripper() {

    shouldGripCone = true;

    gripperSolenoid = new Solenoid(GripperConstants.PCMID, PneumaticsModuleType.CTREPCM, GripperConstants.solenoidID);
  }

  public boolean getShouldGripCone() {
    return this.shouldGripCone;
  }

  public void toggleShouldWeGripACone() {
    this.shouldGripCone = !shouldGripCone;
  }

  public boolean isGripperOpen(){
    return gripperSolenoid.get();
  }

  public Command ToggleGripper(){
    return runOnce(()->{
      gripperSolenoid.toggle();
    });
  }

  public Command changeWhatWeGrip() {
    return runOnce(() -> {
      toggleShouldWeGripACone();
    });
  }
}
