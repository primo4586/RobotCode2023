package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.unmanaged.Unmanaged;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    private final FlywheelSim m_turnMotorSim = new FlywheelSim(
            // Sim Values
            LinearSystemId.identifyVelocitySystem(1, 0.0008), DCMotor.getFalcon500(1), 150/7);

    private final FlywheelSim m_driveMotorSim = new FlywheelSim(
            // Sim Values
            LinearSystemId.identifyVelocitySystem(2.6777,0.45944 ), DCMotor.getFalcon500(1), 8.14);

    private double m_drivePercentOutput;
    private double m_turnPercentOutput;
    private double m_driveMotorSimDistance;
    private double m_turnMotorSimDistance;

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
    angleEncoder.getSimCollection().setRawPosition(0);
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

        simulationPeriodic();
    }

    public void setDesiredStateReveresed(SwerveModuleState desiredState) {
        setDesiredState(new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle), true);
        
        // SmartDashboard.putNumber("desierd velocity",
        // desiredState.speedMetersPerSecond);
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
        // SmartDashboard.putNumber("Mod " + moduleNumber + " Desired Velocity",
        // desiredState.speedMetersPerSecond);

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        mAngleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio));
        if (isOpenLoop) {
            // SmartDashboard.putNumber("Mod " + moduleNumber + " Current Velocity",
            // getState().speedMetersPerSecond);
            // SmartDashboard.putNumber("Mod " + moduleNumber + " Desired Angle ",
            // desiredState.angle.getDegrees());
        }
        lastAngle = angle;

        m_drivePercentOutput = mDriveMotor.getMotorOutputPercent();
        m_turnPercentOutput = mAngleMotor.getMotorOutputPercent();

        SmartDashboard.putNumber("desierd", desiredState.angle.getDegrees());
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset,
                Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        ErrorCode code;
        // code = angleEncoder.configFactoryDefault();
        // System.out.println("Module " + moduleNumber + " CANCoder Factory Default: " +
        // code);
        code = angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
        System.out.println("Module " + moduleNumber + " CANCoder Settings: " + code);
    }

    private void configAngleMotor() {
        ErrorCode code;
        // code = mAngleMotor.configFactoryDefault();
        // System.out.println("Module " + moduleNumber + " Angle Factory Default: " +
        // code);
        code = mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        System.out.println("Module " + moduleNumber + " Angle Settings: " + code);
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        ErrorCode code;
        // code = mDriveMotor.configFactoryDefault();
        // System.out.println("Module " + moduleNumber + " Drive Factory Default: " +
        // code);
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

    public Rotation2d getAngle(){
         return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.angleGearRatio));
    }

    public double getVelocity(){
        return Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
        Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
    }

    public double getOffset() {
        return constants.angleOffset;
    }

    public double getMeterDistance() {
        SmartDashboard.putNumber("drive", mDriveMotor.getSelectedSensorPosition());
        return Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference,
                SwerveConstants.driveGearRatio);
    }

    public SwerveModulePosition getPostion() {
        return new SwerveModulePosition(getMeterDistance(), getState().angle);
    }

    public void simulationPeriodic() {
        m_turnMotorSim.setInputVoltage(m_turnPercentOutput * RobotController.getBatteryVoltage());
        m_driveMotorSim.setInputVoltage(m_drivePercentOutput * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("m_drivePercentOutput", m_drivePercentOutput);
        m_turnMotorSim.update(0.02);
        m_driveMotorSim.update(0.02);

        Unmanaged.feedEnable(20);

        m_turnMotorSimDistance += m_turnMotorSim.getAngularVelocityRadPerSec() * 0.02;
        SmartDashboard.putNumber("m_turnMotorSimDistance", m_turnMotorSimDistance);
        m_driveMotorSimDistance += m_driveMotorSim.getAngularVelocityRadPerSec() * 0.02;
        mAngleMotor
                 .getSimCollection()
                 .setIntegratedSensorRawPosition(
                         (int) (m_turnMotorSimDistance / (360.0 / (2048 * (150 / 7)))));
        mAngleMotor
                .getSimCollection()
                .setIntegratedSensorVelocity(
                        (int) (m_turnMotorSim.getAngularVelocityRadPerSec()
                                / ((360.0 / (2048 * (150 / 7)))*10 )));
        mDriveMotor
                  .getSimCollection()
                  .setIntegratedSensorRawPosition(
                          (int) (m_driveMotorSimDistance / ((Units.inchesToMeters(4) * Math.PI) / (2048 * 8.14))));
        mDriveMotor
                .getSimCollection()
                .setIntegratedSensorVelocity(
                        (int) (m_driveMotorSim.getAngularVelocityRadPerSec()
                                / (((Units.inchesToMeters(4) * Math.PI) / (2048 * 8.14))*10 )));

        

    }
}