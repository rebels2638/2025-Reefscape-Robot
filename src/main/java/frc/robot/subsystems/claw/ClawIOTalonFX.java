package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.pivot.PivotConfigBase;

public class ClawIOTalonFX implements ClawIO {
    private TalonFX clawMotor;

    private final StatusSignal<AngularVelocity> clawVelocityStatusSignal;

    private final StatusSignal<Voltage> clawAppliedVolts;
    private final StatusSignal<Current> clawSupplyCurrent;
    private final StatusSignal<Temperature> clawTemperature;

    @SuppressWarnings("static-access")
    public ClawIOTalonFX(PivotConfigBase config) {
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

        clawMotor = new TalonFX(config.getCANID());
        clawMotor.getConfigurator().apply(clawConfig);

        clawAppliedVolts = clawMotor.getMotorVoltage().clone();
        clawSupplyCurrent = clawMotor.getSupplyCurrent().clone();
        clawTemperature = clawMotor.getDeviceTemp().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                40,
                clawAppliedVolts,
                clawSupplyCurrent,
                clawTemperature
        );

        clawVelocityStatusSignal = clawMotor.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                clawVelocityStatusSignal
        );

        clawMotor.optimizeBusUtilization();
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ClawIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            clawVelocityStatusSignal,
            clawAppliedVolts,
            clawSupplyCurrent,
            clawTemperature
        );

        inputs.clawVelocityRadPerSec = clawVelocityStatusSignal.getValue().in(RadiansPerSecond);

        inputs.clawCurrentDrawAmps = clawSupplyCurrent.getValue().in(Amps);
        inputs.clawAppliedVolts = clawAppliedVolts.getValue().in(Volts);
        inputs.clawTemperatureFahrenheit = clawTemperature.getValue().in(Fahrenheit);

    }

    @Override
    public void setTorqueCurrentFOC(double baseUnitMagnitude) {
        clawMotor.setControl(
            new TorqueCurrentFOC(baseUnitMagnitude)
        );
    }
}