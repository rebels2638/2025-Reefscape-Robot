package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.claw.ClawConfigBase;
import frc.robot.lib.util.Elastic;
import frc.robot.lib.util.PhoenixUtil;

public class ClawIOTalonFX implements ClawIO {
    private final TalonFX clawMotor;
    private final CANrange canRange;

    private final StatusSignal<AngularVelocity> clawVelocityStatusSignal;

    private final StatusSignal<Voltage> clawAppliedVolts;
    private final StatusSignal<Current> clawTorqueCurrent;
    private final StatusSignal<Temperature> clawTemperature;

    private final StatusSignal<Boolean> canRangeIsDetected;

    private final TorqueCurrentFOC clawTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut clawVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final Elastic.Notification canRangeDisconnectAlert = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
                                "Claw CANRange Disconnected", "CANRange Disconnected, claw may not be functioning");

    private final Elastic.Notification motorDisconnectAlert = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
                                "Claw Motor Disconnected", "Motor Disconnected, GOOD LUCK");

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
    private final Debouncer canRangeConnectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
  
    public ClawIOTalonFX(ClawConfigBase config) {
        CANrangeConfiguration canRangeConfiguration = new CANrangeConfiguration();

        canRangeConfiguration.FovParams.FOVCenterX = config.getFOVCenterX();
        canRangeConfiguration.FovParams.FOVCenterY = config.getFOVCenterY();
        canRangeConfiguration.FovParams.FOVRangeX = config.getFOVRangeX();
        canRangeConfiguration.FovParams.FOVRangeY = config.getFOVRangeY();

        canRangeConfiguration.ProximityParams.MinSignalStrengthForValidMeasurement = config.getMinSignalStrengthForValidMeasurement();
        canRangeConfiguration.ProximityParams.ProximityHysteresis = config.getProximityHysteresis();
        canRangeConfiguration.ProximityParams.ProximityThreshold = config.getProximityThreshold();

        canRangeConfiguration.ToFParams.UpdateFrequency = config.getToFUpdateFrequency();
        canRangeConfiguration.ToFParams.UpdateMode = config.getToFUpdateMode();

        canRangeConfiguration.FutureProofConfigs = true;

        canRange = new CANrange(config.getCanRangeCanID(), "drivetrain");
        PhoenixUtil.tryUntilOk( 5, () -> canRange.getConfigurator().apply(canRangeConfiguration, 0.25));

        // pivot motor
        TalonFXConfiguration clawConfig = new TalonFXConfiguration();

        // current and torque limiting
        clawConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        clawConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrentLimit();
        clawConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getSupplyCurrentLimitLowerLimit();
        clawConfig.CurrentLimits.SupplyCurrentLowerTime = config.getSupplyCurrentLimitLowerTime();

        clawConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        clawConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrentLimit();

        clawConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakForwardTorqueCurrent();
        clawConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getPeakReverseTorqueCurrent();

        clawConfig.MotorOutput.NeutralMode = 
            config.isNeutralModeBrake() ? 
                        NeutralModeValue.Brake : 
                        NeutralModeValue.Coast;

        clawConfig.MotorOutput.Inverted = 
            config.isInverted() ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        clawConfig.FutureProofConfigs = true;

        clawMotor = new TalonFX(config.getMotorCanID());
        PhoenixUtil.tryUntilOk(5, () -> clawMotor.getConfigurator().apply(clawConfig, 0.25));

        clawAppliedVolts = clawMotor.getMotorVoltage().clone();
        clawTorqueCurrent = clawMotor.getTorqueCurrent().clone();
        clawTemperature = clawMotor.getDeviceTemp().clone();

        clawVelocityStatusSignal = clawMotor.getVelocity().clone();
        canRangeIsDetected = canRange.getIsDetected().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                clawVelocityStatusSignal,
                canRangeIsDetected,

                clawAppliedVolts,
                clawTorqueCurrent,
                clawTemperature
        );

        clawMotor.optimizeBusUtilization();
        canRange.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.isMotorConnected = 
            motorConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                    clawVelocityStatusSignal,

                    clawAppliedVolts,
                    clawTorqueCurrent,
                    clawTemperature
                ).isOK()
            );
        
        inputs.isCanRangeConnected = 
            canRangeConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                    canRangeIsDetected
                ).isOK()
            );

        inputs.clawVelocityRadPerSec = clawVelocityStatusSignal.getValue().in(RadiansPerSecond);
        inputs.inClaw = canRange.getIsDetected(true).getValue().booleanValue();

        inputs.clawCurrentDrawAmps = clawTorqueCurrent.getValue().in(Amps);
        inputs.clawAppliedVolts = clawAppliedVolts.getValue().in(Volts);
        inputs.clawTemperatureFahrenheit = clawTemperature.getValue().in(Fahrenheit);

        if (!inputs.isCanRangeConnected) {
            Elastic.sendNotification(canRangeDisconnectAlert.withDisplayMilliseconds(10000));
            DriverStation.reportError("Claw CANRange Disconnected", false);
        }

        if (!inputs.isMotorConnected) {
            Elastic.sendNotification(motorDisconnectAlert.withDisplayMilliseconds(10000));
            DriverStation.reportError("Claw Motor Disconnected", false);
        }
    }

    @Override
    public void setTorqueCurrentFOC(double baseUnitMagnitude) {
        clawMotor.setControl(
            clawTorqueRequest.withOutput(baseUnitMagnitude)
        );
    }

    @Override
    public void setVoltage(double baseUnitMagnitude) {
        clawMotor.setControl(
            clawVoltageRequest.withOutput(baseUnitMagnitude)
        );
    }
}