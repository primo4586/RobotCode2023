package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BigArmConstants;

public class BigArm extends SubsystemBase {
  private WPI_TalonSRX bigArmMotor;
  private PIDController bigArmPID;
  private ArmFeedforward bigArmMotorFeedforward;
  private DigitalInput honeSwitch;

  public BigArm() {
    bigArmPID = BigArmConstants.bigArmPID;
    bigArmMotorFeedforward = BigArmConstants.bigArmFeedforward;

    bigArmMotor = new WPI_TalonSRX(BigArmConstants.bigArmMotorPort);

    honeSwitch = new DigitalInput(BigArmConstants.honeSwitchID);
  }

  public void putBigArmInPlace(double setPoint){
    SmartDashboard.putNumber("PID",bigArmPID.calculate(getCurrentArmAngle(), setPoint));
    bigArmMotor.setVoltage(-bigArmPID.calculate(getCurrentArmAngle(), setPoint));
  }

  public Command PutBigArmInPlace(double setPoint){
    return run(()->{
      putBigArmInPlace(setPoint);
    })
    .until(() -> Math.abs(this.getCurrentArmAngle() - setPoint) <= BigArmConstants.ticksTolerance);
  }

  // TODO: Setup conversions according to the encoder's CPR
  public double getCurrentArmAngle() {
    return bigArmMotor.getSelectedSensorPosition();
  }

  public Command setMotorSpeed(DoubleSupplier supplier) {
    return this.run(() -> {

      bigArmMotor.set(supplier.getAsDouble() * 0.5);
    });
  }

  public boolean getHoneSwitch(){
    return !honeSwitch.get();
  }

  public Command Hone(){
    return run(()->{
      bigArmMotor.set(BigArmConstants.honeSpeed);
    })
    .until(()->getHoneSwitch())
    .andThen(()->{bigArmMotor.set(0.0);
      bigArmMotor.setSelectedSensorPosition(BigArmConstants.honeSetPoint);
    });
  }
}
