package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.lib.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;

    // When using followers, the leader motor must keep the DutyCycle, 
    // MotorVoltage, and TorqueCurrent status signals enabled. Additionally, 
    // remote sensors must keep related status signals enabled (such as position and velocity).

    private final StatusSignal<Angle> elevatorPositionStatusSignal1;
    private final StatusSignal<AngularVelocity> elevatorVelocityStatusSignal1;

    private final StatusSignal<Voltage> elevatorAppliedVolts1;
    private final StatusSignal<Current> elevatorTorqueCurrent1;
    private final StatusSignal<Double> elevatorAppliedDutyCycle1;
    private final StatusSignal<Temperature> elevatorTemperature1;

    private final StatusSignal<Angle> elevatorPositionStatusSignal2;
    private final StatusSignal<AngularVelocity> elevatorVelocityStatusSignal2;

    private final StatusSignal<Voltage> elevatorAppliedVolts2;
    private final StatusSignal<Current> elevatorTorqueCurrent2;
    private final StatusSignal<Double> elevatorAppliedDutyCycle2;
    private final StatusSignal<Temperature> elevatorTemperature2;

    private final StatusSignal<Double> elevatorClosedLoopReference1;
    private final StatusSignal<Double> elevatorClosedLoopOutput1;

    private final StatusSignal<Double> elevatorClosedLoopReference2;
    private final StatusSignal<Double> elevatorClosedLoopOutput2;

    private final MotionMagicExpoTorqueCurrentFOC elevatorPositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    private final TorqueCurrentFOC elevatorTorqueRequest = new TorqueCurrentFOC(0);
    private final VoltageOut elevatorVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final double kMAX_HEIGHT_METERS;
    private final double kMIN_HEIGHT_METERS;

    public ElevatorIOTalonFX(ElevatorConfigBase config) {
        kMAX_HEIGHT_METERS = config.getMaxHeightMeters();
        kMIN_HEIGHT_METERS = config.getMinHeightMeters();

        // pivot motor
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        // Motion magic expo
        elevatorConfig.Slot0.kP = config.getKP();
        elevatorConfig.Slot0.kI = config.getKI();
        elevatorConfig.Slot0.kD = config.getKD();
        elevatorConfig.Slot0.kS = config.getKS();
        elevatorConfig.Slot0.kV = config.getKV();
        elevatorConfig.Slot0.kA = config.getKA();
        elevatorConfig.Slot0.kG = config.getKG();

        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        elevatorConfig.MotionMagic.MotionMagicExpo_kA = config.getMotionMagicExpoKA();
        elevatorConfig.MotionMagic.MotionMagicExpo_kV = config.getMotionMagicExpoKV();
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = config.getMotionMagicCruiseVelocityMetersPerSec();

        // encoder
        elevatorConfig.ClosedLoopGeneral.ContinuousWrap = false;
        elevatorConfig.Feedback.SensorToMechanismRatio = config.getMotorToOutputShaftRatio();

        // current and torque limiting
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrentLimit();
        elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getSupplyCurrentLimitLowerLimit();
        elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = config.getSupplyCurrentLimitLowerTime();

        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrentLimit();

        elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakForwardTorqueCurrent();
        elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getPeakReverseTorqueCurrent();

        elevatorConfig.MotorOutput.NeutralMode = 
            config.isNeutralModeBrake() ? 
                        NeutralModeValue.Brake : 
                        NeutralModeValue.Coast;

        elevatorConfig.FutureProofConfigs = true;

        elevatorMotor1 = new TalonFX(config.getCanID1());
        PhoenixUtil.tryUntilOk( 5, () ->
            elevatorMotor1.getConfigurator().apply(
                elevatorConfig.withMotorOutput(
                    elevatorConfig.MotorOutput.withInverted(
                        config.isM1Inverted() ?
                            InvertedValue.Clockwise_Positive :
                            InvertedValue.CounterClockwise_Positive
                    )
                ),
                0.25
            )
        );

        elevatorMotor2 = new TalonFX(config.getCanID2());
        PhoenixUtil.tryUntilOk( 5, () ->
            elevatorMotor2.getConfigurator().apply(
                elevatorConfig.withMotorOutput(
                    elevatorConfig.MotorOutput.withInverted(
                        config.isM2Inverted() ?
                            InvertedValue.Clockwise_Positive :
                            InvertedValue.CounterClockwise_Positive
                    )
                ),
                0.25
            )
        );
        
        elevatorAppliedVolts1 = elevatorMotor1.getMotorVoltage().clone();
        elevatorTorqueCurrent1 = elevatorMotor1.getTorqueCurrent().clone();
        elevatorAppliedDutyCycle1 = elevatorMotor1.getDutyCycle().clone();
        elevatorTemperature1 = elevatorMotor1.getDeviceTemp().clone();

        elevatorAppliedVolts2 = elevatorMotor2.getMotorVoltage().clone();
        elevatorTorqueCurrent2 = elevatorMotor2.getTorqueCurrent().clone();
        elevatorAppliedDutyCycle2 = elevatorMotor2.getDutyCycle().clone();
        elevatorTemperature2 = elevatorMotor2.getDeviceTemp().clone();

        elevatorClosedLoopReference1 = elevatorMotor1.getClosedLoopReference().clone();
        elevatorClosedLoopOutput1 = elevatorMotor1.getClosedLoopOutput().clone();

        elevatorClosedLoopReference2 = elevatorMotor2.getClosedLoopReference().clone();
        elevatorClosedLoopOutput2 = elevatorMotor2.getClosedLoopOutput().clone();

        elevatorPositionStatusSignal1 = elevatorMotor1.getPosition().clone();
        elevatorVelocityStatusSignal1 = elevatorMotor1.getVelocity().clone();

        elevatorPositionStatusSignal2 = elevatorMotor2.getPosition().clone();
        elevatorVelocityStatusSignal2 = elevatorMotor2.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                elevatorAppliedVolts1,
                elevatorTorqueCurrent1,
                elevatorAppliedDutyCycle1,
                elevatorTemperature1,

                elevatorAppliedVolts2,
                elevatorTorqueCurrent2,
                elevatorAppliedDutyCycle2,
                elevatorTemperature2,

                elevatorClosedLoopReference1,
                elevatorClosedLoopOutput1,

                elevatorClosedLoopReference2,
                elevatorClosedLoopOutput2,

                elevatorPositionStatusSignal1,
                elevatorVelocityStatusSignal1,

                elevatorPositionStatusSignal2,
                elevatorVelocityStatusSignal2
        );

        elevatorMotor1.optimizeBusUtilization();
        elevatorMotor2.optimizeBusUtilization();

        elevatorMotor2.setControl(new StrictFollower(config.getCanID1()).withUpdateFreqHz(1000));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            elevatorAppliedVolts1,
            elevatorTorqueCurrent1,
            elevatorAppliedDutyCycle1,
            elevatorTemperature1,

            elevatorAppliedVolts2,
            elevatorTorqueCurrent2,
            elevatorAppliedDutyCycle2,
            elevatorTemperature2,

            elevatorClosedLoopReference1,
            elevatorClosedLoopOutput1,

            elevatorClosedLoopReference2,
            elevatorClosedLoopOutput2,

            elevatorPositionStatusSignal1,
            elevatorVelocityStatusSignal1,

            elevatorPositionStatusSignal2,
            elevatorVelocityStatusSignal2
        );
        
        inputs.elevatorHeightMeters = 
            (BaseStatusSignal.getLatencyCompensatedValue(elevatorPositionStatusSignal1, elevatorVelocityStatusSignal1).in(Rotation) + 
            BaseStatusSignal.getLatencyCompensatedValue(elevatorPositionStatusSignal2, elevatorVelocityStatusSignal2).in(Rotation))
            / 2;

        inputs.elevatorVelocityMetersPerSec = 
            (elevatorVelocityStatusSignal1.getValue().in(RadiansPerSecond) + elevatorVelocityStatusSignal2.getValue().in(RotationsPerSecond)) / 2;

        inputs.elevatorAppliedVolts1 = elevatorAppliedVolts1.getValue().in(Volts);
        inputs.elevatorTorqueCurrent1 = elevatorTorqueCurrent1.getValue().in(Amps);
        inputs.elevatorAppliedDutyCycle1 = elevatorAppliedDutyCycle1.getValue();
        inputs.elevatorTemperatureFahrenheit1 = elevatorTemperature1.getValue().in(Fahrenheit);

        inputs.elevatorAppliedVolts2 = elevatorAppliedVolts2.getValue().in(Volts);
        inputs.elevatorTorqueCurrent2 = elevatorTorqueCurrent2.getValue().in(Amps);
        inputs.elevatorAppliedDutyCycle2 = elevatorAppliedDutyCycle2.getValue();
        inputs.elevatorTemperatureFahrenheit2 = elevatorTemperature2.getValue().in(Fahrenheit);

        Logger.recordOutput("Elevator/elevatorClosedLoopReference1", elevatorClosedLoopReference1.getValueAsDouble());
        Logger.recordOutput("Elevator/elevatorClosedLoopOutput1", elevatorClosedLoopOutput1.getValueAsDouble());

        Logger.recordOutput("Elevator/elevatorClosedLoopReference2", elevatorClosedLoopReference2.getValueAsDouble());
        Logger.recordOutput("Elevator/elevatorClosedLoopOutput2", elevatorClosedLoopOutput2.getValueAsDouble());

    }

    @Override
    public void setHeight(double height) {
        height = MathUtil.clamp(height, kMIN_HEIGHT_METERS, kMAX_HEIGHT_METERS);

        elevatorMotor1.setControl(
            elevatorPositionRequest.withPosition(height)
        ); 
    }

    @Override
    public void setTorqueCurrentFOC(double baseUnitMagnitude) {
        elevatorMotor1.setControl(
            elevatorTorqueRequest.withOutput(baseUnitMagnitude)
        );
    }
    
    @Override
    public void setVoltage(double baseUnitMagnitude) {
        elevatorMotor1.setControl(
            elevatorVoltageRequest.withOutput(baseUnitMagnitude)
        );
    }
}