package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LilConstants;

public class LilArm extends SubsystemBase {
  private WPI_TalonFX lilArmMotor;
  private WPI_TalonSRX lilArmEncoder;

  /** Creates a new LilArm. */
  public LilArm() {

    lilArmEncoder = new WPI_TalonSRX(LilConstants.lilArmEncoderID);

    lilArmMotor = new WPI_TalonFX(LilConstants.lilArmMotorID);

    lilArmEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    setupLilArmMotor();
  }

  public void setupLilArmMotor() {

    lilArmMotor.setInverted(false);
    lilArmMotor.configSupplyCurrentLimit(Constants.ARM_MOTOR_SUPPLY_CONFIG);

    lilArmMotor.setNeutralMode(NeutralMode.Brake);

    lilArmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, LilConstants.kPIDLoopIdx,
        LilConstants.kTimeoutMs);

    lilArmMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

    lilArmMotor.configNeutralDeadband(0.001, LilConstants.kTimeoutMs);

    lilArmMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, LilConstants.kTimeoutMs);
    lilArmMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, LilConstants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    lilArmMotor.configNominalOutputForward(0, LilConstants.kTimeoutMs);
    lilArmMotor.configNominalOutputReverse(0, LilConstants.kTimeoutMs);
    lilArmMotor.configPeakOutputForward(1, LilConstants.kTimeoutMs);
    lilArmMotor.configPeakOutputReverse(-1, LilConstants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    lilArmMotor.selectProfileSlot(LilConstants.kSlotIdx, LilConstants.kPIDLoopIdx);
    lilArmMotor.config_kF(LilConstants.kSlotIdx, LilConstants.lilArmMotorsKF, LilConstants.kTimeoutMs);
    lilArmMotor.config_kP(LilConstants.kSlotIdx, LilConstants.lilArmMotorsKD, LilConstants.kTimeoutMs);
    lilArmMotor.config_kI(LilConstants.kSlotIdx, LilConstants.lilArmMotorsKI, LilConstants.kTimeoutMs);
    lilArmMotor.config_kD(LilConstants.kSlotIdx, LilConstants.lilArmMotorsKD, LilConstants.kTimeoutMs);

    lilArmMotor.configMotionCruiseVelocity(LilConstants.maxSpeed / 1.5, LilConstants.kTimeoutMs);
    lilArmMotor.configMotionAcceleration(LilConstants.maxAcceleration, LilConstants.kTimeoutMs);

    lilArmMotor.setSelectedSensorPosition(
        lilArmEncoder.getSensorCollection().getPulseWidthPosition() / 2 * LilConstants.lilMotorGearRatio,
        LilConstants.kPIDLoopIdx, LilConstants.kTimeoutMs);

    lilArmMotor.configForwardSoftLimitThreshold(LilConstants.softLimitForward);
    lilArmMotor.configReverseSoftLimitThreshold(LilConstants.softLimitReverse);
    lilArmMotor.configForwardSoftLimitEnable(true);
    lilArmMotor.configReverseSoftLimitEnable(true);

  }

  public void zeroEncoderForIntake() {
    this.lilArmEncoder.setSelectedSensorPosition(LilConstants.middleOfRobotSetPoint);
  }

  public void zeroEncoderForAuto() {
    this.lilArmEncoder.setSelectedSensorPosition(LilConstants.autoStartPoint);
  }

  public Command setMotorSpeed(DoubleSupplier speed) {
    return this.run(() -> {

      lilArmMotor.set(speed.getAsDouble());
    });
  }

  public void putArmInPlace(double setpoint) {
    lilArmMotor.set(TalonFXControlMode.MotionMagic, setpoint);
  }

  public Command TurnLilArmToSetpoint(double setpoint) {
    SmartDashboard.putNumber("LilArm Setpoint", setpoint);
    return run(() -> {
      putArmInPlace(setpoint);
    }).until(() -> Math.abs(getCurrentArmPosition() - setpoint) < 3000&&lilArmMotor.getSelectedSensorVelocity()<200);
  }

  public double getCurrentArmPosition() {
    return lilArmMotor.getSelectedSensorPosition();
  }

  public Command zeroLilArm() {
    return runOnce(() -> {
      lilArmMotor.setSelectedSensorPosition(
          lilArmEncoder.getSensorCollection().getPulseWidthPosition() / 2 * LilConstants.lilMotorGearRatio,
          LilConstants.kPIDLoopIdx, LilConstants.kTimeoutMs);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LilArm Position", lilArmMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("LilArm Encoder", lilArmEncoder.getSensorCollection().getPulseWidthPosition());
  }
}
