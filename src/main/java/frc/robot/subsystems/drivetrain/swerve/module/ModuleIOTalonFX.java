package frc.robot.subsystems.drivetrain.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.swerve.SwerveConfigBase;
import frc.robot.constants.swerve.SwerveModuleConfig.GeneralConfig;
import frc.robot.constants.swerve.SwerveModuleConfig.SpecificConfig;
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

    private double modulePosition;
    private double moduleVelocity;
    private Rotation2d moduleAngle;

    private final GeneralConfig generalConfig;
    private final int moduleID;

    @SuppressWarnings("static-access")
    public ModuleIOTalonFX(SwerveConfigBase configBase, SpecificConfig specificConfig, int moduleID) {
        this.generalConfig = configBase.getSharedGeneralConfig();
        this.moduleID = moduleID;

        // TODO: CHECK THIS !!!!!!!!!!!!! :3 
        kDRIVE_MOTOR_ROTATIONS_TO_METERS = generalConfig.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO *
                2 * Math.PI * generalConfig.kDRIVE_WHEEL_RADIUS_METERS;

        kDRIVE_METERS_TO_MOTOR_ROTATIONS = 1 / kDRIVE_MOTOR_ROTATIONS_TO_METERS;

        kSTEER_MOTOR_ROTATIONS_TO_MODULE_ROTATIONS = generalConfig.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;
        kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS = 1
                / generalConfig.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;

        // Drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // Motion magic expo TODO: DIFFRENT ACELL AND DECEL SPEEDS?! CAN BE DONE BY
        // MODNFIING THE ACCEL CONSTANT ON THE FLY
        driveConfig.Slot0.kP = generalConfig.kDRIVE_KP;
        driveConfig.Slot0.kI = generalConfig.kDRIVE_KI;
        driveConfig.Slot0.kD = generalConfig.kDRIVE_KD;
        driveConfig.Slot0.kS = generalConfig.kDRIVE_KS;
        driveConfig.Slot0.kV = generalConfig.kDRIVE_KV;
        driveConfig.Slot0.kA = generalConfig.kDRIVE_KA;
        driveConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        driveConfig.MotionMagic.MotionMagicAcceleration = generalConfig.kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC
                * kDRIVE_METERS_TO_MOTOR_ROTATIONS;

        driveConfig.MotionMagic.MotionMagicJerk = generalConfig.kDRIVE_MOTION_MAGIC_VELOCITY_JERK_METERS_PER_SEC_SEC_SEC
                * kDRIVE_METERS_TO_MOTOR_ROTATIONS;

        // encoder
        driveConfig.ClosedLoopGeneral.ContinuousWrap = generalConfig.kDRIVE_CONTINUOUS_WRAP;
        // driveConfig.Feedback.SensorToMechanismRatio =
        // config.kDRIVE_SENSOR_TO_MECHANISM_RATIO; // TODO: SHOULD WE USE THIS?!

        // Current and torque limiting
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = generalConfig.kDRIVE_SUPPLY_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = generalConfig.kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = generalConfig.kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_TIME;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = generalConfig.kDRIVE_STATOR_CURRENT_LIMIT;

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.kDRIVE_PEAK_FORWARD_TORQUE_CURRENT;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.kDRIVE_PEAK_REVERSE_TORQUE_CURRENT;

        driveMotor = new TalonFX(specificConfig.kDRIVE_CAN_ID, generalConfig.kCAN_BUS_NAME);
        driveMotor.getConfigurator().apply(driveConfig);

        // ABS encoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = generalConfig.kCANCODER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT;
        encoderConfig.MagnetSensor.SensorDirection = generalConfig.kCANCODER_SENSOR_DIRECTION;
        encoderConfig.MagnetSensor.withMagnetOffset(specificConfig.kCANCODER_OFFSET_ROTATIONS);

        steerEncoder = new CANcoder(specificConfig.kCANCODER_CAN_ID, generalConfig.kCAN_BUS_NAME);
        steerEncoder.getConfigurator().apply(encoderConfig);

        // Steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        // Motion magic expo
        steerConfig.Slot0.kP = generalConfig.kSTEER_KP;
        steerConfig.Slot0.kI = generalConfig.kSTEER_KI;
        steerConfig.Slot0.kD = generalConfig.kSTEER_KD;
        steerConfig.Slot0.kS = generalConfig.kSTEER_KS;
        steerConfig.Slot0.kV = generalConfig.kSTEER_KV;
        steerConfig.Slot0.kA = generalConfig.kSTEER_KA;
        steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        steerConfig.MotionMagic.MotionMagicExpo_kA = generalConfig.kSTEER_MOTION_MAGIC_EXPO_KA;
        steerConfig.MotionMagic.MotionMagicExpo_kV = generalConfig.kSTEER_MOTION_MAGIC_EXPO_KV;
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = generalConfig.kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_ROTATIONS_PER_SEC;

        // Cancoder + encoder
        steerConfig.ClosedLoopGeneral.ContinuousWrap = generalConfig.kSTEER_CONTINUOUS_WRAP;
        steerConfig.Feedback.FeedbackRemoteSensorID = specificConfig.kCANCODER_CAN_ID;
        steerConfig.Feedback.FeedbackSensorSource = generalConfig.kSTEER_CANCODER_FEEDBACK_SENSOR_SOURCE;
        steerConfig.Feedback.SensorToMechanismRatio = 1;
        steerConfig.Feedback.RotorToSensorRatio = generalConfig.kSTEER_ROTOR_TO_SENSOR_RATIO;

        // current and torque limiting
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = generalConfig.kSTEER_SUPPLY_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLowerLimit = generalConfig.kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLowerTime = generalConfig.kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_TIME;

        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = generalConfig.kSTEER_STATOR_CURRENT_LIMIT;

        steerConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.kSTEER_PEAK_FORWARD_TORQUE_CURRENT;
        steerConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.kSTEER_PEAK_REVERSE_TORQUE_CURRENT;

        steerMotor = new TalonFX(specificConfig.kSTEER_CAN_ID, generalConfig.kCAN_BUS_NAME);
        steerMotor.getConfigurator().apply(steerConfig);

        // status signals
        driveAppliedVolts = driveMotor.getMotorVoltage().clone();
        driveSupplyCurrent = driveMotor.getSupplyCurrent().clone();
        driveTemperature = driveMotor.getDeviceTemp().clone();

        steerAppliedVolts = steerMotor.getMotorVoltage().clone();
        steerSupplyCurrent = steerMotor.getSupplyCurrent().clone();
        steerTemperature = steerMotor.getDeviceTemp().clone();

        steerEncoderAbsolutePosition = steerEncoder.getAbsolutePosition();

        Phoenix6Odometry.getInstance().registerSignal(driveMotor, driveAppliedVolts);
        Phoenix6Odometry.getInstance().registerSignal(driveMotor, driveSupplyCurrent);
        Phoenix6Odometry.getInstance().registerSignal(driveMotor, driveTemperature);

        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerAppliedVolts);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerSupplyCurrent);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerTemperature);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerEncoderAbsolutePosition);

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
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerEncoderPositionStatusSignal);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        steerEncoder.optimizeBusUtilization();
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ModuleIOInputs inputs) {

        double driveRotations = BaseStatusSignal
                .getLatencyCompensatedValue(drivePositionStatusSignal, driveVelocityStatusSignal).in(Rotation);
        double steerRotations = BaseStatusSignal
                .getLatencyCompensatedValue(steerPositionStatusSignal, steerVelocityStatusSignal).in(Rotation);

        inputs.timestamp = HALUtil.getFPGATime() / 1.0e6;

        inputs.drivePositionMeters = driveRotations * kDRIVE_MOTOR_ROTATIONS_TO_METERS;
        inputs.driveVelocityMetersPerSec = driveVelocityStatusSignal.getValue().in(RotationsPerSecond);

        inputs.driveCurrentDrawAmps = driveSupplyCurrent.getValue().in(Amps);
        inputs.driveAppliedVolts = driveAppliedVolts.getValue().in(Volts);
        inputs.driveTemperatureFahrenheit = driveTemperature.getValue().in(Fahrenheit);

        inputs.steerCANCODERAbsolutePosition = new Rotation2d(steerEncoderAbsolutePosition.getValue().in(Radians));
        inputs.steerPosition = new Rotation2d(
                Units.rotationsToRadians(steerRotations));
        inputs.steerVelocityRadPerSec = steerVelocityStatusSignal.getValue().in(RadiansPerSecond);
        inputs.steerCurrentDrawAmps = steerSupplyCurrent.getValue().in(Amps);
        inputs.steerAppliedVolts = steerAppliedVolts.getValue().in(Volts);
        inputs.steerTemperatureFahrenheit = steerTemperature.getValue().in(Fahrenheit);

        this.moduleAngle = inputs.steerCANCODERAbsolutePosition;
        this.modulePosition = inputs.drivePositionMeters;
        this.moduleVelocity = inputs.driveVelocityMetersPerSec;


    }

    // TODO: validate this later
    // reviewed: correct, however, 2910 uses steerMotor.setControl(new
    // MotionMagicVoltage(0).withPosition(velocityRadSec)); for their stuff
    // TODO: exponential motionamagic profiles

    @Override
    public void setState(SwerveModuleState state) {
        driveMotor.setControl(driveMotorRequest.withVelocity(
                RebelUtil.constrain(
                        state.speedMetersPerSecond,
                        -generalConfig.kDRIVE_MAX_VELOCITY_METERS_PER_SEC,
                        generalConfig.kDRIVE_MAX_VELOCITY_METERS_PER_SEC)
                        ));
                        
        steerMotor.setControl(
                steerMotorRequest.withPosition(
                        state.angle.getRotations())); // TODO: ENSURE THAT THE MODULE HAS PROPER
                                                                              // CONTINUES WRAP!!!!!!
    }

    @Override
    public SwerveModuleState setTargetState(SwerveModuleState state) {
        SwerveModuleState optimizedState = optimize(state, moduleAngle);
        setState(optimizedState);
        return optimizedState;
    }

    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90.0) {
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
      } else {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.moduleVelocity, this.moduleAngle);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.modulePosition, this.moduleAngle);
    }

    @Override
    public void setDriveVoltage(double baseUnitMagnitude) {
    }

}