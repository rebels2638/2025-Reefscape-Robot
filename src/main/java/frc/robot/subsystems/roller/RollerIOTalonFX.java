package frc.robot.subsystems.roller;

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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.roller.RollerConfigBase;

public class RollerIOTalonFX implements RollerIO {
    private TalonFX rollerMotor;

    private final StatusSignal<AngularVelocity> rollerVelocityStatusSignal;

    private final StatusSignal<Voltage> rollerAppliedVolts;
    private final StatusSignal<Current> rollerSupplyCurrent;
    private final StatusSignal<Temperature> rollerTemperature;

    private final TorqueCurrentFOC rollerTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    @SuppressWarnings("static-access")
    public RollerIOTalonFX(RollerConfigBase config) {
        // pivot motor
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        // current and torque limiting
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrentLimit();
        rollerConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getSupplyCurrentLimitLowerLimit();
        rollerConfig.CurrentLimits.SupplyCurrentLowerTime = config.getSupplyCurrentLimitLowerTime();

        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrentLimit();

        rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakForwardTorqueCurrent();
        rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getPeakReverseTorqueCurrent();

        rollerConfig.MotorOutput.NeutralMode = 
            config.isNeutralModeBrake() ? 
                        NeutralModeValue.Brake : 
                        NeutralModeValue.Coast;

        rollerMotor = new TalonFX(config.getCANID());
        rollerMotor.getConfigurator().apply(rollerConfig);

        rollerAppliedVolts = rollerMotor.getMotorVoltage().clone();
        rollerSupplyCurrent = rollerMotor.getSupplyCurrent().clone();
        rollerTemperature = rollerMotor.getDeviceTemp().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                40,
                rollerAppliedVolts,
                rollerSupplyCurrent,
                rollerTemperature
        );

        rollerVelocityStatusSignal = rollerMotor.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                rollerVelocityStatusSignal
        );

        rollerMotor.optimizeBusUtilization();
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(RollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            rollerVelocityStatusSignal,
            rollerAppliedVolts,
            rollerSupplyCurrent,
            rollerTemperature
        );

        inputs.rollerVelocityRadPerSec = rollerVelocityStatusSignal.getValue().in(RadiansPerSecond);

        inputs.rollerCurrentDrawAmps = rollerSupplyCurrent.getValue().in(Amps);
        inputs.rollerAppliedVolts = rollerAppliedVolts.getValue().in(Volts);
        inputs.rollerTemperatureFahrenheit = rollerTemperature.getValue().in(Fahrenheit);

    }

    @Override
    public void setTorqueCurrentFOC(double baseUnitMagnitude) {
        rollerMotor.setControl(
            rollerTorqueRequest.withOutput(baseUnitMagnitude)
        );
    }

    @Override
    public void setVoltage(double baseUnitMagnitude) {
        rollerMotor.setControl(
            rollerVoltageRequest.withOutput(baseUnitMagnitude)
        );
    }
}