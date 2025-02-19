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

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.claw.ClawConfigBase;
import frc.robot.lib.util.Elastic;

public class ClawIOTalonFX implements ClawIO {
    private TalonFX clawMotor;
    private final CANrange canRange;

    private final StatusSignal<AngularVelocity> clawVelocityStatusSignal;
    private final StatusSignal<AngularAcceleration> clawAccelerationStatusSignal;

    private final StatusSignal<Voltage> clawAppliedVolts;
    private final StatusSignal<Current> clawSupplyCurrent;
    private final StatusSignal<Temperature> clawTemperature;

    private final StatusSignal<Boolean> canRangeIsDetected;

    private final TorqueCurrentFOC clawTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut clawVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final Elastic.Notification canRangeDisconnectAlert = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
                                "Claw Disconnected", "CANRange Disconnected, Claw may not be functioning");

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
        canRange.getConfigurator().apply(canRangeConfiguration);

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

        clawConfig.MotorOutput.Inverted = 
            config.getIsMotorInverted() ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        clawConfig.MotorOutput.NeutralMode = 
            config.isNeutralModeBrake() ? 
                        NeutralModeValue.Brake : 
                        NeutralModeValue.Coast;

        clawMotor = new TalonFX(config.getCanID());
        clawMotor.getConfigurator().apply(clawConfig);

        clawAppliedVolts = clawMotor.getMotorVoltage().clone();
        clawSupplyCurrent = clawMotor.getSupplyCurrent().clone();
        clawTemperature = clawMotor.getDeviceTemp().clone();

        clawVelocityStatusSignal = clawMotor.getVelocity().clone();
        clawAccelerationStatusSignal = clawMotor.getAcceleration().clone();
        canRangeIsDetected = canRange.getIsDetected().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                clawAppliedVolts,
                clawSupplyCurrent,
                clawTemperature,
                clawVelocityStatusSignal,
                clawAccelerationStatusSignal
        );

        clawMotor.optimizeBusUtilization();
        canRange.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            clawVelocityStatusSignal,
            canRangeIsDetected,
            clawAppliedVolts,
            clawSupplyCurrent,
            clawTemperature
        );

        double clawVel = BaseStatusSignal.getLatencyCompensatedValue(clawVelocityStatusSignal, clawAccelerationStatusSignal).in(RadiansPerSecond);

        inputs.clawVelocityRadPerSec = clawVel;
        inputs.inClaw = canRange.getIsDetected(true).getValue().booleanValue();

        inputs.clawCurrentDrawAmps = clawSupplyCurrent.getValue().in(Amps);
        inputs.clawAppliedVolts = clawAppliedVolts.getValue().in(Volts);
        inputs.clawTemperatureFahrenheit = clawTemperature.getValue().in(Fahrenheit);

        if (canRange.isConnected(0.1)) {
            Elastic.sendNotification(canRangeDisconnectAlert.withDisplayMilliseconds(10000));
            DriverStation.reportError("Roller CANRange Disconnected", true);
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