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
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.roller.RollerConfigBase;

public class RollerIOSparkMax implements RollerIO {
    private final SparkMax rollerMotor;
    private final CANrange canRange;

    private final StatusSignal<Boolean> canRangeIsDetected;

    private final TorqueCurrentFOC rollerTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final RollerConfigBase config;

    public RollerIOSparkMax(RollerConfigBase config) {
        this.config = config;

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

        canRange = new CANrange(config.getCanRangeCanID());
        canRange.getConfigurator().apply(canRangeConfiguration);

        rollerMotor = new SparkMax(config.getRollerMotorCanID(), SparkLowLevel.MotorType.kBrushless);

        canRangeIsDetected = canRange.getIsDetected().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                canRangeIsDetected
        );

        canRange.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(canRangeIsDetected);

        inputs.rollerVelocityRadPerSec = rollerMotor.getEncoder().getVelocity();
        inputs.inRoller = canRangeIsDetected.getValue().booleanValue();

        inputs.rollerCurrentDrawAmps = rollerMotor.getOutputCurrent();
        inputs.rollerAppliedVolts = rollerMotor.getAppliedOutput() * 12;
        inputs.rollerTemperatureFahrenheit = rollerMotor.getMotorTemperature() * 9/5 + 32;
    }

    @Override
    public void setVoltage(double baseUnitMagnitude) {
        rollerMotor.setVoltage(baseUnitMagnitude);
    }
}