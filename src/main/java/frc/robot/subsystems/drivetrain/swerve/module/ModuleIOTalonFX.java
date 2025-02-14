package frc.robot.subsystems.drivetrain.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.swerve.moduleConfigs.SwerveModuleGeneralConfigBase;
import frc.robot.constants.swerve.moduleConfigs.SwerveModuleSpecificConfigBase;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.drivetrain.swerve.Phoenix6Odometry;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    private final StatusSignal<Angle> drivePositionStatusSignal;
    private final StatusSignal<AngularVelocity> driveVelocityStatusSignal;
    private final StatusSignal<AngularAcceleration> driveAccelerationStatusSignal;

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

    private final SwerveModuleGeneralConfigBase generalConfig;
    private final int moduleID;

    private double currentDriveVelo;

    public ModuleIOTalonFX(SwerveModuleGeneralConfigBase generalConfig, SwerveModuleSpecificConfigBase specificConfig, int moduleID) {
        this.generalConfig = generalConfig;
        this.moduleID = moduleID;

        // Drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // Motion magic expo TODO: DIFFRENT ACELL AND DECEL SPEEDS?! CAN BE DONE BY
        // MODNFIING THE ACCEL CONSTANT ON THE FLY
        driveConfig.Slot0.kP = generalConfig.getDriveKP();
        driveConfig.Slot0.kI = generalConfig.getDriveKI();
        driveConfig.Slot0.kD = generalConfig.getDriveKD();
        driveConfig.Slot0.kS = generalConfig.getDriveKS();
        driveConfig.Slot0.kV = generalConfig.getDriveKV();
        driveConfig.Slot0.kA = generalConfig.getDriveKA();
        driveConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        driveConfig.MotionMagic.MotionMagicAcceleration = generalConfig.getDriveMotionMagicVelocityAccelerationMetersPerSecSec();
        driveConfig.MotionMagic.MotionMagicJerk = generalConfig.getDriveMotionMagicVelocityJerkMetersPerSecSecSec();

        // Cancoder + encoder
        driveConfig.ClosedLoopGeneral.ContinuousWrap = false;
        driveConfig.Feedback.SensorToMechanismRatio = 
            generalConfig.getDriveMotorToOutputShaftRatio() /
            (generalConfig.getDriveWheelRadiusMeters() * 2 * Math.PI);

        driveConfig.MotorOutput.NeutralMode = 
            generalConfig.getIsDriveNeutralModeBrake() ? 
                NeutralModeValue.Brake : 
                NeutralModeValue.Coast;

        driveConfig.MotorOutput.Inverted = 
            specificConfig.getIsDriveInverted() ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = generalConfig.getDriveSupplyCurrentLimit();
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = generalConfig.getDriveSupplyCurrentLimit();
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = generalConfig.getDriveSupplyCurrentLimitLowerTime();

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = generalConfig.getDriveStatorCurrentLimit();

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.getDrivePeakForwardTorqueCurrent();
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.getDrivePeakReverseTorqueCurrent();

        driveConfig.FutureProofConfigs = true;
        
        driveMotor = new TalonFX(specificConfig.getDriveCanId(), generalConfig.getCanBusName());
        driveMotor.getConfigurator().apply(driveConfig);

        // ABS encoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = generalConfig.getCancoderAbsoluteSensorDiscontinuityPoint();
        encoderConfig.MagnetSensor.SensorDirection = generalConfig.getCancoderSensorDirection();
        encoderConfig.MagnetSensor.withMagnetOffset(specificConfig.getCancoderOffsetRotations());

        encoderConfig.FutureProofConfigs = true;

        steerEncoder = new CANcoder(specificConfig.getCancoderCanId(), generalConfig.getCanBusName());
        steerEncoder.getConfigurator().apply(encoderConfig);

        // Steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        // Motion magic expo
        steerConfig.Slot0.kP = generalConfig.getSteerKP();
        steerConfig.Slot0.kI = generalConfig.getSteerKI();
        steerConfig.Slot0.kD = generalConfig.getSteerKD();
        steerConfig.Slot0.kS = generalConfig.getSteerKS();
        steerConfig.Slot0.kV = generalConfig.getSteerKV();
        steerConfig.Slot0.kA = generalConfig.getSteerKA();
        steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        steerConfig.MotionMagic.MotionMagicExpo_kA = generalConfig.getSteerMotionMagicExpoKA();
        steerConfig.MotionMagic.MotionMagicExpo_kV = generalConfig.getSteerMotionMagicExpoKV();
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = generalConfig.getSteerMotionMagicCruiseVelocityRotationsPerSec();

        steerConfig.MotorOutput.NeutralMode = 
            generalConfig.getIsSteerNeutralModeBrake() ? 
                NeutralModeValue.Brake : 
                NeutralModeValue.Coast;

        steerConfig.MotorOutput.Inverted = 
            specificConfig.getIsSteerInverted() ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;
                
        // Cancoder + encoder
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.Feedback.FeedbackRemoteSensorID = specificConfig.getCancoderCanId();
        steerConfig.Feedback.FeedbackSensorSource = generalConfig.getSteerCancoderFeedbackSensorSource();
        steerConfig.Feedback.SensorToMechanismRatio = 1;
        steerConfig.Feedback.RotorToSensorRatio = generalConfig.getSteerRotorToSensorRatio();

        // current and torque limiting
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = generalConfig.getSteerSupplyCurrentLimit();
        steerConfig.CurrentLimits.SupplyCurrentLowerLimit = generalConfig.getSteerSupplyCurrentLimitLowerLimit();
        steerConfig.CurrentLimits.SupplyCurrentLowerTime = generalConfig.getSteerSupplyCurrentLimitLowerTime();

        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = generalConfig.getSteerStatorCurrentLimit();

        steerConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.getSteerPeakForwardTorqueCurrent();
        steerConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.getSteerPeakReverseTorqueCurrent();

        steerConfig.FutureProofConfigs = true;

        steerMotor = new TalonFX(specificConfig.getSteerCanId(), generalConfig.getCanBusName());
        steerMotor.getConfigurator().apply(steerConfig);

        // status signals
        driveAppliedVolts = driveMotor.getMotorVoltage().clone();
        driveSupplyCurrent = driveMotor.getSupplyCurrent().clone();
        driveTemperature = driveMotor.getDeviceTemp().clone();

        steerAppliedVolts = steerMotor.getMotorVoltage().clone();
        steerSupplyCurrent = steerMotor.getSupplyCurrent().clone();
        steerTemperature = steerMotor.getDeviceTemp().clone();

        steerEncoderAbsolutePosition = steerEncoder.getAbsolutePosition().clone();
        steerEncoderPositionStatusSignal = steerEncoder.getPosition().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            driveAppliedVolts,
            driveSupplyCurrent,
            driveTemperature,

            steerAppliedVolts,
            steerSupplyCurrent,
            steerTemperature
        );

        drivePositionStatusSignal = driveMotor.getPosition().clone();
        driveVelocityStatusSignal = driveMotor.getVelocity().clone();
        driveAccelerationStatusSignal = driveMotor.getAcceleration().clone();

        steerPositionStatusSignal = steerMotor.getPosition().clone();
        steerVelocityStatusSignal = steerMotor.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            drivePositionStatusSignal,
            driveVelocityStatusSignal,
            driveAccelerationStatusSignal,

            steerPositionStatusSignal,
            steerVelocityStatusSignal,

            steerEncoderAbsolutePosition,
            steerEncoderPositionStatusSignal
        );

        Phoenix6Odometry.getInstance().registerSignal(driveMotor, drivePositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(driveMotor, driveVelocityStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, driveAccelerationStatusSignal);

        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerPositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(steerMotor, steerVelocityStatusSignal);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        steerEncoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            driveAppliedVolts,
            driveSupplyCurrent,
            driveTemperature,

            steerAppliedVolts,
            steerSupplyCurrent,
            steerTemperature,

            steerEncoderAbsolutePosition,
            steerEncoderPositionStatusSignal
        );

        double drivePosition = BaseStatusSignal
                .getLatencyCompensatedValue(drivePositionStatusSignal, driveVelocityStatusSignal).in(Rotation);

        double steerRotations = BaseStatusSignal
                .getLatencyCompensatedValue(steerPositionStatusSignal, steerVelocityStatusSignal).in(Rotation);

        inputs.timestamp = (drivePositionStatusSignal.getTimestamp().getTime() + steerPositionStatusSignal.getTimestamp().getTime()) / 2;

        inputs.drivePositionMeters = drivePosition;
        inputs.driveVelocityMetersPerSec = driveVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.driveAccelerationMetersPerSecSec = driveAccelerationStatusSignal.getValue().in(RotationsPerSecondPerSecond);

        inputs.steerPosition = new Rotation2d(
                Units.rotationsToRadians(steerRotations));
        inputs.steerVelocityRadPerSec = steerVelocityStatusSignal.getValue().in(RadiansPerSecond);

        inputs.driveCurrentDrawAmps = driveSupplyCurrent.getValue().in(Amps);
        inputs.driveAppliedVolts = driveAppliedVolts.getValue().in(Volts);
        inputs.driveTemperatureFahrenheit = driveTemperature.getValue().in(Fahrenheit);

        inputs.steerCurrentDrawAmps = steerSupplyCurrent.getValue().in(Amps);
        inputs.steerAppliedVolts = steerAppliedVolts.getValue().in(Volts);
        inputs.steerTemperatureFahrenheit = steerTemperature.getValue().in(Fahrenheit);

        Logger.recordOutput("SwerveDrive/module" + moduleID + "/driveClosedLoopReference", driveMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("SwerveDrive/module" + moduleID + "/driveClosedLoopOutput", driveMotor.getClosedLoopOutput().getValueAsDouble());

        currentDriveVelo = inputs.driveVelocityMetersPerSec;
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveMotor.setControl(driveMotorRequest.withVelocity(
            RebelUtil.constrain(
                state.speedMetersPerSecond,
                -generalConfig.getDriveMaxVelocityMetersPerSec(),
                generalConfig.getDriveMaxVelocityMetersPerSec())
            ).
            withAcceleration(
                Math.abs(state.speedMetersPerSecond) >= Math.abs(currentDriveVelo) ?
                generalConfig.getDriveMotionMagicVelocityAccelerationMetersPerSecSec() :
                generalConfig.getDriveMotionMagicVelocityDecelerationMetersPerSecSec()
            )
        );
        
        steerMotor.setControl(
            steerMotorRequest.withPosition(
                state.angle.getRotations()
            )
        );
    }

    @Override
    public void setDriveVoltage(double baseUnitMagnitude) {
    }

}