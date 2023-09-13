package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BigConstants;

public class BigArm extends SubsystemBase {
  private WPI_TalonFX bigArmMotor;
  private DigitalInput homeSwitch;

  public BigArm() {
    bigArmMotor = new WPI_TalonFX(BigConstants.bigArmMotorID);
    homeSwitch = new DigitalInput(BigConstants.homeSwitchID);
    setupBigArmMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Big Arm Position", bigArmMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Big Arm error", bigArmMotor.getClosedLoopError());
    SmartDashboard.putBoolean("Home Switch value", homeSwitch.get());



  }

  public void setupBigArmMotor() {
    bigArmMotor.setInverted(false);
    bigArmMotor.configSupplyCurrentLimit(Constants.ARM_MOTOR_SUPPLY_CONFIG);

    bigArmMotor.setNeutralMode(NeutralMode.Brake);
    

    bigArmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, BigConstants.kPIDLoopIdx,
      BigConstants.kTimeoutMs);

    bigArmMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

    bigArmMotor.configNeutralDeadband(0.001, BigConstants.kTimeoutMs);

    bigArmMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, BigConstants.kTimeoutMs);
    bigArmMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, BigConstants.kTimeoutMs);

    bigArmMotor.configNominalOutputForward(0, BigConstants.kTimeoutMs);
    bigArmMotor.configNominalOutputReverse(0, BigConstants.kTimeoutMs);
    bigArmMotor.configPeakOutputForward(1, BigConstants.kTimeoutMs);
    bigArmMotor.configPeakOutputReverse(-1, BigConstants.kTimeoutMs);

    bigArmMotor.selectProfileSlot(BigConstants.kSlotIdx, BigConstants.kPIDLoopIdx);
    bigArmMotor.config_kF(BigConstants.kSlotIdx, BigConstants.bigArmKF, BigConstants.kTimeoutMs);
    bigArmMotor.config_kP(BigConstants.kSlotIdx, BigConstants.bigArmKP, BigConstants.kTimeoutMs);
    bigArmMotor.config_kI(BigConstants.kSlotIdx, BigConstants.bigArmKI, BigConstants.kTimeoutMs);
    bigArmMotor.config_kD(BigConstants.kSlotIdx, BigConstants.bigArmKD, BigConstants.kTimeoutMs);

    bigArmMotor.configMotionCruiseVelocity(BigConstants.maxSpeed/3.5, BigConstants.kTimeoutMs);
    bigArmMotor.configMotionAcceleration(BigConstants.maxAcceleration, BigConstants.kTimeoutMs);
  }

  public void zeroEncoderForMiddleOfBot() {
    bigArmMotor.setSelectedSensorPosition(BigConstants.intakeSetPoint);
  }

  public void putArmInPlace(double setpoint) {
    bigArmMotor.set(TalonFXControlMode.MotionMagic, setpoint);
  }

  public Command TurnBigArmToSetpoint(double setpoint) {
    SmartDashboard.putNumber("bigArm Setpoint", setpoint);
    return run(() -> {
      putArmInPlace(setpoint);
    }).until(()->Math.abs(getCurrentArmPosition()-setpoint)<1500&&bigArmMotor.getSelectedSensorVelocity()<200);
  }

  public double getCurrentArmPosition() {
    return bigArmMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public Command setMotorSpeed(DoubleSupplier speed) {
    return this.run(() -> {
      bigArmMotor.set(speed.getAsDouble());
    });
  }

  public Command Home() {
    return run(() -> {
      bigArmMotor.set(BigConstants.homeSpeed);
    })
        .until(() -> !homeSwitch.get())
        .andThen(() -> {
          bigArmMotor.set(0.0);
          bigArmMotor.getSensorCollection().setIntegratedSensorPosition(BigConstants.homeSetPoint,30);
        });
  }

  public Command zeroEncoder() {
    return runOnce(() -> bigArmMotor.setSelectedSensorPosition(0));
  }
}
