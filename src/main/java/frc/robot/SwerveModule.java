package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    private SwerveModuleConstants constants;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS,
            Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        this.constants = moduleConstants;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();
        configAngleEncoder();
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();
        configAngleMotor();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
        configDriveMotor();
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
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

        if (isOpenLoop) {
            // If it's open loop, it means we don't use PID/FF, and we are not aiming for
            // accuracy
            double percentOutput = (desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed)
                    * Constants.SwerveConstants.maxPercentVelocity;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            // Conversion from meter per second to falcon tick speeds.
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));

        }
        SmartDashboard.putNumber("Mod " + moduleNumber + " Desired Velocity", desiredState.speedMetersPerSecond);

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        mAngleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio));
        if (isOpenLoop) {
            SmartDashboard.putNumber("Mod " + moduleNumber + " Current Velocity", getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + moduleNumber + " Desired Angle ", desiredState.angle.getDegrees());
        }
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset,
                Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        ErrorCode code;
        code = angleEncoder.configFactoryDefault();
        System.out.println("Module " + moduleNumber + " CANCoder Factory Default: " + code);
        code = angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
        System.out.println("Module " + moduleNumber + " CANCoder Settings: " + code);
    }

    private void configAngleMotor() {
        ErrorCode code;
        code = mAngleMotor.configFactoryDefault();
        System.out.println("Module " + moduleNumber + " Angle Factory Default: " + code);
        code = mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        System.out.println("Module " + moduleNumber + " Angle Settings: " + code);
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        ErrorCode code;
        code = mDriveMotor.configFactoryDefault();
        System.out.println("Module " + moduleNumber + " Drive Factory Default: " + code);
        code = mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        System.out.println("Module " + moduleNumber + " Angle Settings: " + code);
        mDriveMotor.setInverted(constants.driveInvert);
        mDriveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        code = mDriveMotor.setSelectedSensorPosition(0);
        System.out.println("Module " + moduleNumber + " Drive Selected Sensor Position: " + code);
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
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public double getOffset() {
        return constants.angleOffset;
    }

    public double getMeterDistance() {
        return Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference,
                SwerveConstants.driveGearRatio);
    }

    public SwerveModulePosition getPostion() {
        return new SwerveModulePosition(getMeterDistance(), getState().angle);
    }

    public void configLockForAngle() {
       mAngleMotor.setNeutralMode(NeutralMode.Brake);
    }
}