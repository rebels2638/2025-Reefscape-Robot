package frc.robot.subsystems.roller;

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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.roller.RollerConfigBase;

public class RollerIOTalonFX implements RollerIO {
    private final TalonFX rollerMotor;
    private final CANrange canRange;

    private final StatusSignal<AngularVelocity> rollerVelocityStatusSignal;

    private final StatusSignal<Voltage> rollerAppliedVolts;
    private final StatusSignal<Current> rollerSupplyCurrent;
    private final StatusSignal<Temperature> rollerTemperature;

    private final StatusSignal<Boolean> canRangeIsDetected;


    private final TorqueCurrentFOC rollerTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    public RollerIOTalonFX(RollerConfigBase config) {
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

        rollerConfig.FutureProofConfigs = true;

        rollerMotor = new TalonFX(config.getRollerMotorCanID());
        rollerMotor.getConfigurator().apply(rollerConfig);

        rollerAppliedVolts = rollerMotor.getMotorVoltage().clone();
        rollerSupplyCurrent = rollerMotor.getSupplyCurrent().clone();
        rollerTemperature = rollerMotor.getDeviceTemp().clone();

        rollerVelocityStatusSignal = rollerMotor.getVelocity().clone();
        canRangeIsDetected = canRange.getIsDetected().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                rollerVelocityStatusSignal,
                canRangeIsDetected,

                rollerAppliedVolts,
                rollerSupplyCurrent,
                rollerTemperature
        );

        rollerMotor.optimizeBusUtilization();
        canRange.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            rollerVelocityStatusSignal,
            canRangeIsDetected,

            rollerAppliedVolts,
            rollerSupplyCurrent,
            rollerTemperature
        );

        inputs.rollerVelocityRadPerSec = rollerVelocityStatusSignal.getValue().in(RadiansPerSecond);
        inputs.inRoller = canRange.getIsDetected(true).getValue().booleanValue();

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