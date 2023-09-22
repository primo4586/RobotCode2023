// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CTREModuleState;
import frc.lib.util.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule {
    
    public int moduleNumber;
    private double angleOffset;
    private double lastAngle;
    private SwerveModuleConstants moduleConstants;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS,
        SwerveConstants.driveKV);

    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    private CANSparkMax mAngleMotor;
    private SparkMaxPIDController angleController;
    private RelativeEncoder integratedAngleEncoder;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        this.moduleConstants = moduleConstants;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();
        //configAngleEncoder();
        //configAngleEncoder();

        /* Angle Motor Config */
        //Timer.delay(1.0);
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();
        //configAngleMotor();
        //configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
        //configDriveMotor();
        //configDriveMotor();

        lastAngle = getState().angle.getDegrees();

    }

    // public void runCharacterization(double volts) {

    //     mDriveMotor.setVoltage(volts);
    // }

    public double getCharacterizationVelocity() {
        return Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
        Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio)/Units.inchesToMeters(4);
    }

    public void align(){
        angleController.setReference(0, ControlType.kPosition);
    }

    /**
     * Drives the swerve module to be at the desired state
     * 
     * @param desiredState The desired state the module should be in (speed and
     *                     angle setpoints)
     * @param isOpenLoop   If we should use PID and FF or not.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // Custom optimize command, since
                                                                                 // default WPILib optimize assumes
                                                                                 // continuous controller which CTRE is
                                                                                 // not
        
        SmartDashboard.putNumber("desierdSpeed", Math.abs(desiredState.speedMetersPerSecond));
        if (isOpenLoop) {
            // If it's open loop, it means we don't use PID/FF, and we are not aiming for
            // accuracy
            double percentOutput = (desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed)
                    * Constants.SwerveConstants.maxPercentVelocity;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            // Conversion from meter per second to falcon tick speeds.
            double velocity = (desiredState.speedMetersPerSecond*Constants.SwerveConstants.driveGearRatio)/Constants.SwerveConstants.wheelCircumference;
            mDriveMotor.set(ControlMode.Velocity, velocity*2048/10, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond)/12);

        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : (desiredState.angle.getDegrees()); // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        angleController.setReference(angle, ControlType.kPosition);
        lastAngle = angle;
    }
    
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    /**
     * Returns the state of the module (current velocity and angle) in a
     * SwerveModuleState object
     * 
     * @return SwerveModuleState object
     */
    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        return new SwerveModuleState(velocity, angle);
    }

    public double getMeterDistance() {
        return Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference,
                SwerveConstants.driveGearRatio);
    }

    public SwerveModulePosition getPostion() {
        return new SwerveModulePosition(getMeterDistance(), getState().angle);
    }

    public boolean areWheelsAligned() {
        return Math.abs(getCanCoder().getDegrees() - angleOffset) < 2;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getEncoder().getPosition(),
                Constants.SwerveConstants.angleGearRatio));
    }

    public double getVelocity() {
        return Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
                Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
    }

    // module config don't put anything after
    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset;
        System.out.println(integratedAngleEncoder.setPosition(absolutePosition).toString());
    }

    private void configAngleEncoder() {
        CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration(); /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        
        ErrorCode code;
        code = angleEncoder.configFactoryDefault();
        System.out.println("Module " + moduleNumber + " CANCoder Factory Default: " + code);
        code = angleEncoder.configAllSettings(swerveCanCoderConfig);
        System.out.println("Module " + moduleNumber + " CANCoder Settings: " + code);
    }

    private void configAngleMotor() {
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        //System.out.println(mAngleMotor.restoreFactoryDefaults().toString()+"reset");
        System.out.println(mAngleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit).toString()+"current");
        mAngleMotor.setInverted(SwerveConstants.angleMotorInvert);
        System.out.println(mAngleMotor.setIdleMode(SwerveConstants.angleNeutralMode)+"neutral");
        System.out.println(angleController.setP(SwerveConstants.angleKP).toString()+"p");
        System.out.println(angleController.setI(SwerveConstants.angleKI).toString()+"i");
        System.out.println(angleController.setD(SwerveConstants.angleKD).toString()+"d");
        System.out.println(angleController.setFF(SwerveConstants.angleKF).toString()+"f");
        angleController.setOutputRange(-1, 1);
        System.out.println(mAngleMotor.enableVoltageCompensation(SwerveConstants.voltageComp).toString()+"v comp");
        System.out.println(integratedAngleEncoder.setPositionConversionFactor(360/SwerveConstants.angleGearRatio).toString()+"pose");
        System.out.println(mAngleMotor.burnFlash().toString()+"burn");
        Timer.delay(1.0);
        resetToAbsolute();
        resetToAbsolute();
        resetToAbsolute();
    }

    private void configDriveMotor() {
        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

        /* Swerve Drive Motor Configuration */
        swerveDriveFXConfig.slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = SwerveConstants.driveKD;
        swerveDriveFXConfig.slot0.kF = SwerveConstants.driveKF;
        swerveDriveFXConfig.supplyCurrLimit = SwerveConstants.driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.voltageCompSaturation = 12;

        ErrorCode code;
        code = mDriveMotor.configFactoryDefault();
        System.out.println("Module " + moduleNumber + " Drive Factory Default: " + code);
        code = mDriveMotor.configAllSettings(swerveDriveFXConfig);
        System.out.println("Module " + moduleNumber + " Angle Settings: " + code);
        mDriveMotor.setInverted(moduleConstants.driveInvert);
        mDriveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
        code = mDriveMotor.setSelectedSensorPosition(0);
        System.out.println("Module " + moduleNumber + " Drive Selected Sensor Position: " + code);
    }
}
