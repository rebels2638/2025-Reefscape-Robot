package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.pivot.PivotConfigBase;
import frc.robot.lib.util.PhoenixUtil;
import frc.robot.lib.util.RebelTrapezoidProfile;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.claw.Claw;
import frc.robot.lib.util.Elastic;

public class PivotIOTalonFX implements PivotIO {
    private TalonFX pivotMotor;

    private final StatusSignal<Angle> pivotPositionStatusSignal;
    private final StatusSignal<AngularVelocity> pivotVelocityStatusSignal;

    private final StatusSignal<Voltage> pivotAppliedVolts;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotTemperature;

    private final StatusSignal<Double> pivotClosedLoopReference;
    private final StatusSignal<Double> pivotClosedLoopOutput;

    private final MotionMagicExpoTorqueCurrentFOC pivotPositionRequest = 
        new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    private final TorqueCurrentFOC pivotTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final double kMAX_ANGLE_ROTATIONS;
    private final double kMIN_ANGLE_ROTATIONS;
    private final double kSTARTING_ANGLE_RAD;

    private final Debouncer pivotConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    private final Elastic.Notification pivotDisconnectAlert = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
        "Pivot Motor Disconnected", "Pivot Disconnected, GOOD LUCK");

    private final PivotConfigBase config;
    private final TalonFXConfiguration pivotConfig;

    private boolean lastInClaw = false;

    public PivotIOTalonFX(PivotConfigBase config) {
        this.config = config;
        kMAX_ANGLE_ROTATIONS = config.getMaxAngleRotations();
        kMIN_ANGLE_ROTATIONS = config.getMinAngleRotations();
        kSTARTING_ANGLE_RAD = config.getStartingAngleRotations() * Math.PI * 2;

        // pivot motor
        pivotConfig = new TalonFXConfiguration();

        // Motion magic expo
        //fuck
        pivotConfig.Slot0.kP = config.getKP();
        pivotConfig.Slot0.kI = config.getKI();
        pivotConfig.Slot0.kD = config.getKD();
        pivotConfig.Slot0.kS = config.getKS();
        pivotConfig.Slot0.kV = config.getKV();
        pivotConfig.Slot0.kA = config.getKA();
        pivotConfig.Slot0.kG = config.getKG();

        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        pivotConfig.MotionMagic.MotionMagicExpo_kA = config.getMotionMagicExpoKA();
        pivotConfig.MotionMagic.MotionMagicExpo_kV = config.getMotionMagicExpoKV();
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = config.getMotionMagicCruiseVelocityRotationsPerSec();

        // encoder
        pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
        pivotConfig.Feedback.SensorToMechanismRatio = config.getMotorToOutputShaftRatio();
        
        // current and torque limiting
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrentLimit();
        pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getSupplyCurrentLimitLowerLimit();
        pivotConfig.CurrentLimits.SupplyCurrentLowerTime = config.getSupplyCurrentLimitLowerTime();

        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrentLimit();

        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakForwardTorqueCurrent();
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getPeakReverseTorqueCurrent();

        pivotConfig.MotorOutput.NeutralMode = 
            config.isNeutralModeBrake() ? 
                        NeutralModeValue.Brake : 
                        NeutralModeValue.Coast;

        pivotConfig.MotorOutput.Inverted = 
            config.isInverted() ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        pivotConfig.FutureProofConfigs = true;

        pivotMotor = new TalonFX(config.getCanID());
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));

        pivotAppliedVolts = pivotMotor.getMotorVoltage().clone();
        pivotTorqueCurrent = pivotMotor.getTorqueCurrent().clone();
        pivotTemperature = pivotMotor.getDeviceTemp().clone();

        pivotPositionStatusSignal = pivotMotor.getPosition().clone();
        pivotVelocityStatusSignal = pivotMotor.getVelocity().clone();

        pivotClosedLoopReference = pivotMotor.getClosedLoopReference().clone();
        pivotClosedLoopOutput = pivotMotor.getClosedLoopOutput().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                pivotAppliedVolts,
                pivotTorqueCurrent,
                pivotTemperature,
                pivotPositionStatusSignal,
                pivotVelocityStatusSignal,
                pivotClosedLoopReference,
                pivotClosedLoopOutput
        );

        pivotMotor.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotMotorConnected = 
            pivotConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                    pivotPositionStatusSignal,
                    pivotVelocityStatusSignal,
                    pivotAppliedVolts,
                    pivotTorqueCurrent,
                    pivotTemperature,
                    pivotClosedLoopReference,
                    pivotClosedLoopOutput
                ).isOK()
        );
        
        inputs.pivotPosition = new Rotation2d(
            BaseStatusSignal.getLatencyCompensatedValue(pivotPositionStatusSignal, pivotVelocityStatusSignal).in(Radians) +
            kSTARTING_ANGLE_RAD
        );

        inputs.pivotVelocityRadPerSec = pivotVelocityStatusSignal.getValue().in(RadiansPerSecond);

        inputs.pivotCurrentDrawAmps = pivotTorqueCurrent.getValue().in(Amps);
        inputs.pivotAppliedVolts = pivotAppliedVolts.getValue().in(Volts);
        inputs.pivotTemperatureFahrenheit = pivotTemperature.getValue().in(Fahrenheit);

        Logger.recordOutput("Pivot/pivotClosedLoopReference", Math.PI * 2 * pivotClosedLoopReference.getValueAsDouble() + kSTARTING_ANGLE_RAD);
        Logger.recordOutput("Pivot/pivotClosedLoopOutput", pivotClosedLoopOutput.getValueAsDouble());

        if (!inputs.pivotMotorConnected) {
            Elastic.sendNotification(pivotDisconnectAlert.withDisplayMilliseconds(10000));
            DriverStation.reportError("Pivot Motor Disconnected", false);
        }
    }

    @Override
    public void setAngle(Rotation2d state) {
        if (Claw.getInstance().inClaw() && !lastInClaw) {
            pivotMotor.getConfigurator().apply(pivotConfig.MotionMagic.withMotionMagicCruiseVelocity(0.3));
            lastInClaw = true;
        }
        else if (!Claw.getInstance().inClaw() && lastInClaw) {
            pivotMotor.getConfigurator().apply(pivotConfig.MotionMagic.withMotionMagicCruiseVelocity(3));
            lastInClaw = false;
        }
        // can't do this
        pivotMotor.setControl(
            pivotPositionRequest.withPosition(
                RebelUtil.constrain(
                    state.getRotations(),
                    kMIN_ANGLE_ROTATIONS,
                    kMAX_ANGLE_ROTATIONS
                ) - kSTARTING_ANGLE_RAD / (2 * Math.PI)
            ).withSlot(Claw.getInstance().inClaw() ? 0 : 1)
        ); 
    }

    @Override
    public void setTorqueCurrentFOC(double baseUnitMagnitude) {
        pivotMotor.setControl(
            pivotTorqueRequest.withOutput(baseUnitMagnitude)
        );
    }

    @Override
    public void setVoltage(double baseUnitMagnitude) {
        pivotMotor.setControl(
            pivotVoltageRequest.withOutput(baseUnitMagnitude)
        );
    }
}