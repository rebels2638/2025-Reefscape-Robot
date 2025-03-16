package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.climber.ClimberConfigBase;
import frc.robot.lib.util.PhoenixUtil;
import frc.robot.lib.util.RebelUtil;
import frc.robot.lib.util.Elastic;

public class ClimberIOTalonFX implements ClimberIO {
    private TalonFX climberMotor;

    private final StatusSignal<Angle> climberPositionStatusSignal;
    private final StatusSignal<AngularVelocity> climberVelocityStatusSignal;

    private final StatusSignal<Voltage> climberAppliedVolts;
    private final StatusSignal<Current> climberTorqueCurrent;
    private final StatusSignal<Temperature> climberTemperature;

    private final StatusSignal<Double> climberClosedLoopReference;
    private final StatusSignal<Double> climberClosedLoopOutput;

    private final MotionMagicExpoTorqueCurrentFOC climberPositionRequest = 
        new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    private final TorqueCurrentFOC climberTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut climberVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final double kMAX_ANGLE_ROTATIONS;
    private final double kMIN_ANGLE_ROTATIONS;
    private final double kSTARTING_ANGLE_RAD;

    private final Debouncer climberConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    private final Elastic.Notification climberDisconnectAlert = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
        "Climber Motor Disconnected", "Climber Disconnected, GOOD LUCK");

    private final ClimberConfigBase config; 
    public ClimberIOTalonFX(ClimberConfigBase config) {
        this.config = config;
        kMAX_ANGLE_ROTATIONS = config.getMaxAngleRotations();
        kMIN_ANGLE_ROTATIONS = config.getMinAngleRotations();
        kSTARTING_ANGLE_RAD = config.getStartingAngleRotations() * Math.PI * 2;

        // climber motor
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();

        // Motion magic expo
        //fuck
        climberConfig.Slot0.kP = config.getKP();
        climberConfig.Slot0.kI = config.getKI();
        climberConfig.Slot0.kD = config.getKD();
        climberConfig.Slot0.kS = config.getKS();
        climberConfig.Slot0.kV = config.getKV();
        climberConfig.Slot0.kA = config.getKA();
        climberConfig.Slot0.kG = config.getKG();

        climberConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        climberConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        climberConfig.MotionMagic.MotionMagicExpo_kA = config.getMotionMagicExpoKA();
        climberConfig.MotionMagic.MotionMagicExpo_kV = config.getMotionMagicExpoKV();
        climberConfig.MotionMagic.MotionMagicCruiseVelocity = config.getMotionMagicCruiseVelocityRotationsPerSec();

        // encoder
        climberConfig.ClosedLoopGeneral.ContinuousWrap = false;
        climberConfig.Feedback.SensorToMechanismRatio = config.getMotorToOutputShaftRatio();
        
        // current and torque limiting
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrentLimit();
        climberConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getSupplyCurrentLimitLowerLimit();
        climberConfig.CurrentLimits.SupplyCurrentLowerTime = config.getSupplyCurrentLimitLowerTime();

        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrentLimit();

        climberConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakForwardTorqueCurrent();
        climberConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getPeakReverseTorqueCurrent();

        climberConfig.MotorOutput.NeutralMode = 
            config.isNeutralModeBrake() ? 
                        NeutralModeValue.Brake : 
                        NeutralModeValue.Coast;

        climberConfig.MotorOutput.Inverted = 
            config.isInverted() ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        climberConfig.FutureProofConfigs = true;

        climberMotor = new TalonFX(config.getCanID());
        PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfig, 0.25));

        climberAppliedVolts = climberMotor.getMotorVoltage().clone();
        climberTorqueCurrent = climberMotor.getTorqueCurrent().clone();
        climberTemperature = climberMotor.getDeviceTemp().clone();

        climberPositionStatusSignal = climberMotor.getPosition().clone();
        climberVelocityStatusSignal = climberMotor.getVelocity().clone();

        climberClosedLoopReference = climberMotor.getClosedLoopReference().clone();
        climberClosedLoopOutput = climberMotor.getClosedLoopOutput().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                climberAppliedVolts,
                climberTorqueCurrent,
                climberTemperature,
                climberPositionStatusSignal,
                climberVelocityStatusSignal,
                climberClosedLoopReference,
                climberClosedLoopOutput
        );

        climberMotor.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberMotorConnected = 
            climberConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                    climberPositionStatusSignal,
                    climberVelocityStatusSignal,
                    climberAppliedVolts,
                    climberTorqueCurrent,
                    climberTemperature,
                    climberClosedLoopReference,
                    climberClosedLoopOutput
                ).isOK()
        );
        
        inputs.climberPosition = new Rotation2d(
            BaseStatusSignal.getLatencyCompensatedValue(climberPositionStatusSignal, climberVelocityStatusSignal).in(Radians) +
            kSTARTING_ANGLE_RAD
        );

        inputs.climberVelocityRadPerSec = climberVelocityStatusSignal.getValue().in(RadiansPerSecond);

        inputs.climberCurrentDrawAmps = climberTorqueCurrent.getValue().in(Amps);
        inputs.climberAppliedVolts = climberAppliedVolts.getValue().in(Volts);
        inputs.climberTemperatureFahrenheit = climberTemperature.getValue().in(Fahrenheit);

        Logger.recordOutput("climber/climberClosedLoopReference", Math.PI * 2 * climberClosedLoopReference.getValueAsDouble() + kSTARTING_ANGLE_RAD);
        Logger.recordOutput("climber/climberClosedLoopOutput", climberClosedLoopOutput.getValueAsDouble());

        if (!inputs.climberMotorConnected) {
            Elastic.sendNotification(climberDisconnectAlert.withDisplayMilliseconds(10000));
            DriverStation.reportError("Climber Motor Disconnected", false);
        }

    }

    @Override
    public void setAngle(Rotation2d state) {
        climberMotor.setControl(
            climberPositionRequest.withPosition(
                RebelUtil.constrain(
                    state.getRotations(),
                    kMIN_ANGLE_ROTATIONS,
                    kMAX_ANGLE_ROTATIONS
                ) - kSTARTING_ANGLE_RAD / (2 * Math.PI)
            )
        ); 
    }

    @Override
    public void setTorqueCurrentFOC(double baseUnitMagnitude) {
        climberMotor.setControl(
            climberTorqueRequest.withOutput(baseUnitMagnitude)
        );
    }

    @Override
    public void setVoltage(double baseUnitMagnitude) {
        climberMotor.setControl(
            climberVoltageRequest.withOutput(baseUnitMagnitude)
        );
    }
}