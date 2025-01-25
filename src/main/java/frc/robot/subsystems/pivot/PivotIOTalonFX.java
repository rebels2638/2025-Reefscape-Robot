package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.pivot.PivotConfigBase;
import frc.robot.subsystems.drivetrain.swerve.Phoenix6Odometry;

public class PivotIOTalonFX implements PivotIO {
    private TalonFX pivotMotor;

    private final StatusSignal<Angle> pivotPositionStatusSignal;
    private final StatusSignal<AngularVelocity> pivotVelocityStatusSignal;

    private final StatusSignal<Voltage> pivotAppliedVolts;
    private final StatusSignal<Current> pivotSupplyCurrent;
    private final StatusSignal<Temperature> pivotTemperature;

    private final MotionMagicExpoTorqueCurrentFOC pivotMotorRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    private final PivotConfigBase config;

    @SuppressWarnings("static-access")
    public PivotIOTalonFX(PivotConfigBase config) {
        this.config = config;

        // pivot motor
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        // Motion magic expo
        pivotConfig.Slot0.kP = config.getKP();
        pivotConfig.Slot0.kI = config.getKI();
        pivotConfig.Slot0.kD = config.getKD();
        pivotConfig.Slot0.kS = config.getKS();
        pivotConfig.Slot0.kV = config.getKV();
        pivotConfig.Slot0.kA = config.getKA();
        pivotConfig.Slot0.kG = config.getKG();

        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

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

        pivotMotor = new TalonFX(config.getCANID());
        pivotMotor.getConfigurator().apply(pivotConfig);

        pivotAppliedVolts = pivotMotor.getMotorVoltage().clone();
        pivotSupplyCurrent = pivotMotor.getSupplyCurrent().clone();
        pivotTemperature = pivotMotor.getDeviceTemp().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pivotAppliedVolts,
                pivotSupplyCurrent,
                pivotTemperature
        );

        pivotPositionStatusSignal = pivotMotor.getPosition().clone();
        pivotVelocityStatusSignal = pivotMotor.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pivotPositionStatusSignal,
                pivotVelocityStatusSignal
        );

        Phoenix6Odometry.getInstance().registerSignal(pivotMotor, pivotPositionStatusSignal);
        Phoenix6Odometry.getInstance().registerSignal(pivotMotor, pivotVelocityStatusSignal);

        pivotMotor.optimizeBusUtilization();
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(PivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            pivotPositionStatusSignal,
            pivotVelocityStatusSignal,
            pivotAppliedVolts,
            pivotSupplyCurrent,
            pivotTemperature
        );
        
        double pivotRotations = BaseStatusSignal
                .getLatencyCompensatedValue(pivotPositionStatusSignal, pivotVelocityStatusSignal).in(Rotation);

        inputs.pivotPositionRad = new Rotation2d(
                Units.rotationsToRadians(pivotRotations));
        inputs.pivotVelocityRadPerSec = pivotVelocityStatusSignal.getValue().in(RadiansPerSecond);

        inputs.pivotCurrentDrawAmps = pivotSupplyCurrent.getValue().in(Amps);
        inputs.pivotAppliedVolts = pivotAppliedVolts.getValue().in(Volts);
        inputs.pivotTemperatureFahrenheit = pivotTemperature.getValue().in(Fahrenheit);

    }

    @Override
    public void setAngle(Rotation2d state) {
        pivotMotor.setControl(
            pivotMotorRequest.withPosition(state.getRotations())
        ); 
    }

    @Override
    public void setVoltage(double baseUnitMagnitude) {
        pivotMotor.setControl(
            new VoltageOut(baseUnitMagnitude)
        );
    }
}