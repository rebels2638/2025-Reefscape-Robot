package frc.robot.subsystems.drivetrain.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SwerveModuleConfigs.SwerveModuleConfig;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.drivetrain.swerve.Phoenix6Odometry;

public class ModuleIOTalonFX implements ModuleIO {
    private TalonFX driveMotor;

    private TalonFX steerMotor;
    private CANcoder steerEncoder;


    private final StatusSignal<Angle> drivePositionStatusSignal;
    private final StatusSignal<AngularVelocity> driveVelocityStatusSignal;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveSupplyCurrent;
    private final StatusSignal<Temperature> driveTemperature;

    private final StatusSignal<Angle> steerPositionStatusSignal;
    private final StatusSignal<AngularVelocity> steerVelocityStatusSignal;
    private final StatusSignal<Voltage> steerAppliedVolts;
    private final StatusSignal<Current> steerSupplyCurrent;
    private final StatusSignal<Temperature> steerTemperature;
    private final StatusSignal<Angle> steerEncoderPositionStatusSignal;
    private final StatusSignal<Angle> steerEncoderAbsolutePosition;

    private final MotionMagicVelocityTorqueCurrentFOC driveMotorRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    private final MotionMagicExpoTorqueCurrentFOC steerMotorRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    private final double kDRIVE_MOTOR_ROTATIONS_TO_METERS; 
    private final double kDRIVE_METERS_TO_MOTOR_ROTATIONS; 

    private final double kSTEER_MOTOR_ROTATIONS_TO_MODULE_ROTATIONS;
    private final double kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS;

    private final SwerveModuleConfig config;
    @SuppressWarnings("static-access")
    public ModuleIOTalonFX(SwerveModuleConfig config) {
        this.config = config;

        // TODO: CHECK THIS !!!!!!!!!!!!!
        kDRIVE_MOTOR_ROTATIONS_TO_METERS =
            config.kGENERAL_CONFIG.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO * 
            2 * Math.PI * config.kGENERAL_CONFIG.kDRIVE_WHEEL_RADIUS_METERS;

        kDRIVE_METERS_TO_MOTOR_ROTATIONS = 1 / kDRIVE_MOTOR_ROTATIONS_TO_METERS;

        kSTEER_MOTOR_ROTATIONS_TO_MODULE_ROTATIONS = config.kGENERAL_CONFIG.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;
        kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS =  1 / config.kGENERAL_CONFIG.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;

        // Drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // Motion magic expo TODO: DIFFRENT ACELL AND DECEL SPEEDS?! CAN BE DONE BY MODNFIING THE ACCEL CONSTANT ON THE FLY
        driveConfig.Slot0.kP = config.kGENERAL_CONFIG.kDRIVE_KP;
        driveConfig.Slot0.kI = config.kGENERAL_CONFIG.kDRIVE_KI;
        driveConfig.Slot0.kD = config.kGENERAL_CONFIG.kDRIVE_KD;
        driveConfig.Slot0.kS = config.kGENERAL_CONFIG.kDRIVE_KS;
        driveConfig.Slot0.kV = config.kGENERAL_CONFIG.kDRIVE_KV;
        driveConfig.Slot0.kA = config.kGENERAL_CONFIG.kDRIVE_KA;

        driveConfig.MotionMagic.MotionMagicAcceleration = 
            config.kGENERAL_CONFIG.kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC * kDRIVE_METERS_TO_MOTOR_ROTATIONS;

        driveConfig.MotionMagic.MotionMagicJerk =
            config.kGENERAL_CONFIG.kDRIVE_MOTION_MAGIC_VELOCITY_JERK_METERS_PER_SEC_SEC_SEC * kDRIVE_METERS_TO_MOTOR_ROTATIONS;

        // encoder 
        driveConfig.ClosedLoopGeneral.ContinuousWrap = config.kGENERAL_CONFIG.kDRIVE_CONTINUOUS_WRAP;
        // driveConfig.Feedback.SensorToMechanismRatio = config.kDRIVE_SENSOR_TO_MECHANISM_RATIO; // TODO: SHOULD WE USE THIS?!

        // Current and torque limiting
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = config.kGENERAL_CONFIG.kDRIVE_SUPPLY_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = config.kGENERAL_CONFIG.kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = config.kGENERAL_CONFIG.kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_TIME;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = config.kGENERAL_CONFIG.kDRIVE_STATOR_CURRENT_LIMIT;

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.kGENERAL_CONFIG.kDRIVE_PEAK_FORWARD_TORQUE_CURRENT;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.kGENERAL_CONFIG.kDRIVE_PEAK_REVERSE_TORQUE_CURRENT;

        driveMotor = new TalonFX(config.kSPECIFIC_CONFIG.kDRIVE_CAN_ID, config.kGENERAL_CONFIG.kCAN_BUS_NAME);
        driveMotor.getConfigurator().apply(driveConfig);

        // ABS encoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = config.kGENERAL_CONFIG.kCANCODER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT;
        encoderConfig.MagnetSensor.SensorDirection = config.kGENERAL_CONFIG.kCANCODER_SENSOR_DIRECTION;
        encoderConfig.MagnetSensor.withMagnetOffset(config.kSPECIFIC_CONFIG.kCANCODER_OFFSET_ROTATIONS);

        steerEncoder = new CANcoder(config.kSPECIFIC_CONFIG.kCANCODER_CAN_ID, config.kGENERAL_CONFIG.kCAN_BUS_NAME);
        steerEncoder.getConfigurator().apply(encoderConfig);

        // Steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        // Motion magic expo
        steerConfig.Slot0.kP = config.kGENERAL_CONFIG.kSTEER_KP;
        steerConfig.Slot0.kI = config.kGENERAL_CONFIG.kSTEER_KI;
        steerConfig.Slot0.kD = config.kGENERAL_CONFIG.kSTEER_KD;
        steerConfig.Slot0.kS = config.kGENERAL_CONFIG.kSTEER_KS;
        steerConfig.Slot0.kV = config.kGENERAL_CONFIG.kSTEER_KV;
        steerConfig.Slot0.kA = config.kGENERAL_CONFIG.kSTEER_KA;

        steerConfig.MotionMagic.MotionMagicExpo_kA = config.kGENERAL_CONFIG.kSTEER_MOTION_MAGIC_EXPO_KA;
        steerConfig.MotionMagic.MotionMagicExpo_kV = config.kGENERAL_CONFIG.kSTEER_MOTION_MAGIC_EXPO_KV;
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 
            config.kGENERAL_CONFIG.kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC * kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS;

        // Cancoder + encoder 
        steerConfig.ClosedLoopGeneral.ContinuousWrap = config.kGENERAL_CONFIG.kSTEER_CONTINUOUS_WRAP;
        steerConfig.Feedback.FeedbackRemoteSensorID = config.kSPECIFIC_CONFIG.kCANCODER_CAN_ID;
        steerConfig.Feedback.FeedbackSensorSource = config.kGENERAL_CONFIG.kSTEER_CANCODER_FEEDBACK_SENSOR_SOURCE;
        // steerConfig.Feedback.SensorToMechanismRatio = config.kSTEER_SENSOR_TO_MECHANISM_RATIO;
        steerConfig.Feedback.RotorToSensorRatio = config.kGENERAL_CONFIG.kSTEER_ROTOR_TO_SENSOR_RATIO;
        
        // current and torque limiting
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = config.kGENERAL_CONFIG.kSTEER_SUPPLY_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLowerLimit = config.kGENERAL_CONFIG.kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLowerTime = config.kGENERAL_CONFIG.kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_TIME;

        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = config.kGENERAL_CONFIG.kSTEER_STATOR_CURRENT_LIMIT;

        steerConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.kGENERAL_CONFIG.kSTEER_PEAK_FORWARD_TORQUE_CURRENT;
        steerConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.kGENERAL_CONFIG.kSTEER_PEAK_REVERSE_TORQUE_CURRENT;

        steerMotor = new TalonFX(config.kSPECIFIC_CONFIG.kSTEER_CAN_ID, config.kGENERAL_CONFIG.kCAN_BUS_NAME);
        steerMotor.getConfigurator().apply(steerConfig);
        
        // status signals
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTemperature = driveMotor.getDeviceTemp();

        steerAppliedVolts = steerMotor.getMotorVoltage();
        steerSupplyCurrent = steerMotor.getSupplyCurrent();
        steerTemperature = steerMotor.getDeviceTemp();

        steerEncoderAbsolutePosition = steerEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                driveAppliedVolts,
                driveSupplyCurrent,
                steerAppliedVolts,
                steerSupplyCurrent,
                driveTemperature,
                steerTemperature,
                steerEncoderAbsolutePosition);

        drivePositionStatusSignal = driveMotor.getPosition().clone();
        driveVelocityStatusSignal = driveMotor.getVelocity().clone();
        steerPositionStatusSignal = steerMotor.getPosition().clone();
        steerVelocityStatusSignal = steerMotor.getVelocity().clone();
        steerEncoderPositionStatusSignal = steerEncoder.getPosition().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                drivePositionStatusSignal,
                driveVelocityStatusSignal,
                steerPositionStatusSignal,
                steerVelocityStatusSignal,
                steerEncoderPositionStatusSignal);

        Phoenix6Odometry.getInstance().registerSignal(driveMotor, drivePositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(driveMotor, driveVelocityStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerPositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerVelocityStatusSignal);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        steerEncoder.optimizeBusUtilization();
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ModuleIOInputs inputs) {

        double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePositionStatusSignal, driveVelocityStatusSignal).in(Rotation);
        double steerRotations = BaseStatusSignal.getLatencyCompensatedValue(steerPositionStatusSignal, steerVelocityStatusSignal).in(Rotation);

        BaseStatusSignal.refreshAll(
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTemperature,
                steerAppliedVolts,
                steerSupplyCurrent,
                steerTemperature,
                steerEncoderAbsolutePosition);

        inputs.timestamp = HALUtil.getFPGATime() / 1.0e6;

        inputs.drivePositionMeters = driveRotations * kDRIVE_MOTOR_ROTATIONS_TO_METERS;
        inputs.driveVelocityMetersPerSec = driveVelocityStatusSignal.getValue().in(RotationsPerSecond) * kDRIVE_MOTOR_ROTATIONS_TO_METERS;

        inputs.driveCurrentDrawAmps = driveSupplyCurrent.getValue().in(Amps);
        inputs.driveAppliedVolts = driveAppliedVolts.getValue().in(Volts);
        inputs.driveTemperatureFahrenheit = driveTemperature.getValue().in(Fahrenheit);

        inputs.steerCANCODERAbsolutePosition = new Rotation2d(steerEncoderAbsolutePosition.getValue().in(Radians));
        inputs.steerPosition = new Rotation2d(Units.rotationsToRadians(steerRotations) * kSTEER_MOTOR_ROTATIONS_TO_MODULE_ROTATIONS);
        inputs.steerVelocityRadPerSec = steerVelocityStatusSignal.getValue().in(RadiansPerSecond) * kSTEER_MOTOR_ROTATIONS_TO_MODULE_ROTATIONS;

        inputs.steerCurrentDrawAmps = steerSupplyCurrent.getValue().in(Amps);
        inputs.steerAppliedVolts = steerAppliedVolts.getValue().in(Volts);
        inputs.steerTemperatureFahrenheit = steerTemperature.getValue().in(Fahrenheit);

    }

    //TODO: validate this later
    // reviewed: correct, however, 2910 uses steerMotor.setControl(new MotionMagicVoltage(0).withPosition(velocityRadSec)); for their stuff
    // TODO: exponential motionamagic profiles

    @Override
    public void setState(SwerveModuleState state) {
        driveMotor.setControl(driveMotorRequest.withVelocity(
            RebelUtil.constrain(
                state.speedMetersPerSecond,
                -config.kGENERAL_CONFIG.kDRIVE_MAX_VELOCITY_METERS_PER_SEC,
                config.kGENERAL_CONFIG.kDRIVE_MAX_VELOCITY_METERS_PER_SEC
            ) * 
            kDRIVE_METERS_TO_MOTOR_ROTATIONS
        ));
        steerMotor.setControl(steerMotorRequest.withPosition(state.angle.getRotations() * kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS)); // TODO: ENSURE THAT THE MODULE HAS PROPER CONTINUES WRAP!!!!!!
    }

    @Override
    public void setDriveVoltage(double baseUnitMagnitude) {}

}