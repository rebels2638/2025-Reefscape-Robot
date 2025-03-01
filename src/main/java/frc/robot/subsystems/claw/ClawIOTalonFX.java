package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.claw.ClawConfigBase;
import frc.robot.lib.util.PhoenixUtil;

public class ClawIOTalonFX implements ClawIO {
    private TalonFX clawMotor;

    private final StatusSignal<AngularVelocity> clawVelocityStatusSignal;
    private final StatusSignal<AngularAcceleration> clawAccelerationStatusSignal;

    private final StatusSignal<Voltage> clawAppliedVolts;
    private final StatusSignal<Current> clawTorqueCurrent;
    private final StatusSignal<Temperature> clawTemperature;

    private final TorqueCurrentFOC clawTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut clawVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    public ClawIOTalonFX(ClawConfigBase config) {
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
        PhoenixUtil.tryUntilOk( 5, () -> clawMotor.getConfigurator().apply(clawConfig, 0.25));

        clawAppliedVolts = clawMotor.getMotorVoltage().clone();
        clawTorqueCurrent = clawMotor.getTorqueCurrent().clone();
        clawTemperature = clawMotor.getDeviceTemp().clone();

        clawVelocityStatusSignal = clawMotor.getVelocity().clone();
        clawAccelerationStatusSignal = clawMotor.getAcceleration().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                clawAppliedVolts,
                clawTorqueCurrent,
                clawTemperature,
                clawVelocityStatusSignal,
                clawAccelerationStatusSignal
        );

        clawMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            clawVelocityStatusSignal,
            clawAppliedVolts,
            clawTorqueCurrent,
            clawTemperature
        );

        double clawVel = BaseStatusSignal.getLatencyCompensatedValue(clawVelocityStatusSignal, clawAccelerationStatusSignal).in(RadiansPerSecond);

        inputs.clawVelocityRadPerSec = clawVel;

        inputs.clawCurrentDrawAmps = clawTorqueCurrent.getValue().in(Amps);
        inputs.clawAppliedVolts = clawAppliedVolts.getValue().in(Volts);
        inputs.clawTemperatureFahrenheit = clawTemperature.getValue().in(Fahrenheit);

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