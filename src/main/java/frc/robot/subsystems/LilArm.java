package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LilConstants;

public class LilArm extends SubsystemBase {
  private TalonFX lilArmMotor;
  private WPI_TalonSRX lilArmEncoder;
  private MotionMagicVoltage motionMagicVoltage;

  /** Creates a new LilArm. */
  public LilArm() {

    lilArmEncoder = new WPI_TalonSRX(LilConstants.lilArmEncoderID);

    lilArmMotor = new TalonFX(00);

    lilArmEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    motionMagicVoltage = new MotionMagicVoltage(0, true, LilConstants.lilArmKV, 0, false);

    setupLilArmMotor();
  }

  public void setupLilArmMotor() {
    lilArmMotor.setInverted(false);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kP = LilConstants.lilArmKP;
    motorConfig.Slot0.kI = LilConstants.lilArmKI;
    motorConfig.Slot0.kD = LilConstants.lilArmKD;
    motorConfig.Slot0.kV = LilConstants.lilArmKV;
    motorConfig.Slot0.kS = LilConstants.lilArmKS;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = LilConstants.maxSpeed;
    motorConfig.MotionMagic.MotionMagicAcceleration = LilConstants.maxAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = LilConstants.maxJerk;
    motionMagicVoltage.Slot = 0;

    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.armEnableCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = Constants.armContinuousCurrentLimit;
    motorConfig.CurrentLimits.SupplyTimeThreshold = Constants.armPeakCurrentDuration;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.armPeakCurrentLimit;

    lilArmMotor.getConfigurator().apply(motorConfig);

    lilArmMotor.setRotorPosition(0);
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

  public Command TurnLilArmToSetpoint(double setpoint) {
    SmartDashboard.putNumber("LilArm Setpoint", setpoint);
    return runOnce(() -> {
      lilArmMotor.setControl(motionMagicVoltage.withPosition(setpoint));
    });
  }

  public double getCurrentArmPosition() {
    return lilArmMotor.getRotorPosition().getValue();
  }

  public Command zeroLilArm() {
    return runOnce(() -> {
      lilArmMotor.setRotorPosition(0);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LilArm Position", lilArmEncoder.getSelectedSensorPosition());
  }
}
