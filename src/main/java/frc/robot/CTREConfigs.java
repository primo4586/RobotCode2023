package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// Builds & holds all static configurations for the module motors & cancoders
public final class CTREConfigs {
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;


    public CTREConfigs(){
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
        driveSupplyLimit.SupplyCurrentThreshold = Constants.SwerveConstants.drivePeakCurrentLimit;
        driveSupplyLimit.SupplyTimeThreshold = Constants.SwerveConstants.drivePeakCurrentDuration;
        driveSupplyLimit.SupplyCurrentLimit = Constants.SwerveConstants.driveContinuousCurrentLimit;

        OpenLoopRampsConfigs openLoopRamp = new OpenLoopRampsConfigs();
        openLoopRamp.DutyCycleOpenLoopRampPeriod = 0.25;

        swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.driveKD;
        swerveDriveFXConfig.Slot0.kV = Constants.SwerveConstants.driveKF;        
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;
        swerveDriveFXConfig.OpenLoopRamps = openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps = Constants.SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}