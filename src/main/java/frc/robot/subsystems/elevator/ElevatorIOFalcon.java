package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorIOFalcon implements ElevatorIO {
    private TalonFX m_motor2;
    private TalonFX m_motor1;

    private final StatusSignal<Angle> m1PositionStatusSignal;
    private final StatusSignal<AngularVelocity> m1VelocityStatusSignal;
    private final StatusSignal<Voltage> m1AppliedVolts;
    private final StatusSignal<Current> m1SupplyCurrent;
    private final StatusSignal<Temperature> m1Temperature;

    private final StatusSignal<Angle> m2PositionStatusSignal;
    private final StatusSignal<AngularVelocity> m2VelocityStatusSignal;
    private final StatusSignal<Voltage> m2AppliedVolts;
    private final StatusSignal<Current> m2SupplyCurrent;
    private final StatusSignal<Temperature> m2Temperature;

    private final MotionMagicExpoTorqueCurrentFOC driveMotorRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    private final double kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO = 0.0;
    private final double k_FIRST_STAGE_TO_SECOND = 0.0;
    private final double k_SECOND_STAGE_TO_THIRD = 0.0;
    private final double k_SPROCKET_DIAMETER = 0.0;
    private final double k_MAX_HEIGHT = 0.0;
    private final double k_MIN_HEIGHT = 0.0;
    private final double k_TOLERANCE_METERS = 0.0;

    private double elevatorPosition;
    private double elevatorVelocity;
    private double desiredPosition;

    private double dt;

    @SuppressWarnings("static-access")
    public ElevatorIOFalcon() {

        dt = Timer.getTimestamp();

        m_motor1 = new TalonFX(0);
        m_motor2 = new TalonFX(0);

        // status signals
        m1PositionStatusSignal = m_motor1.getPosition().clone();
        m1VelocityStatusSignal = m_motor1.getVelocity().clone();
        m1AppliedVolts = m_motor1.getMotorVoltage().clone();
        m1SupplyCurrent = m_motor1.getSupplyCurrent().clone();
        m1Temperature = m_motor1.getDeviceTemp().clone();

        m2PositionStatusSignal = m_motor2.getPosition().clone();
        m2VelocityStatusSignal = m_motor2.getVelocity().clone();
        m2AppliedVolts = m_motor2.getMotorVoltage().clone();
        m2SupplyCurrent = m_motor2.getSupplyCurrent().clone();
        m2Temperature = m_motor2.getDeviceTemp().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                m1PositionStatusSignal,
                m1VelocityStatusSignal,
                m2PositionStatusSignal,
                m2VelocityStatusSignal);

        m_motor1.optimizeBusUtilization();
        m_motor2.optimizeBusUtilization();
    }

    @Override
    @SuppressWarnings("static-access")
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {

        dt = Timer.getTimestamp() - dt;

        double m1Rotations =  BaseStatusSignal
                .getLatencyCompensatedValue(m1PositionStatusSignal, m1VelocityStatusSignal).in(Rotation);
        double m2Rotations = BaseStatusSignal
                .getLatencyCompensatedValue(m2PositionStatusSignal, m2VelocityStatusSignal).in(Rotation);

        inputs.timestamp = HALUtil.getFPGATime() / 1.0e6;

        inputs.m1PositionMeters = m1Rotations * kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO;
        inputs.m1VelocityMetersPerSec = m1VelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.m1CurrentDrawAmps = m1SupplyCurrent.getValue().in(Amps);
        inputs.m1AppliedVolts = m1AppliedVolts.getValue().in(Volts);
        inputs.m1TemperatureFahrenheit = m1Temperature.getValue().in(Fahrenheit);

        inputs.m2PositionMeters = m2Rotations * kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO;
        inputs.m2VelocityMetersPerSec = m2VelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.m2CurrentDrawAmps = m2SupplyCurrent.getValue().in(Amps);
        inputs.m2AppliedVolts = m2AppliedVolts.getValue().in(Volts);
        inputs.m2TemperatureFahrenheit = m2Temperature.getValue().in(Fahrenheit);

        inputs.reachedSetpoint = MathUtil.isNear(this.desiredPosition, elevatorPosition, k_TOLERANCE_METERS);

        this.elevatorPosition = (m1PositionStatusSignal.getValueAsDouble() + m2PositionStatusSignal.getValueAsDouble())/2 
                            * k_SPROCKET_DIAMETER * Math.PI * kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO * k_FIRST_STAGE_TO_SECOND * k_SECOND_STAGE_TO_THIRD;
        this.elevatorVelocity = this.elevatorPosition/dt;

    }

    @Override
    public void setPosition(double position) {
        position = MathUtil.clamp(position, k_MIN_HEIGHT, k_MAX_HEIGHT);
        this.desiredPosition = position;

        m_motor1.setControl(driveMotorRequest.withPosition(position));            
        m_motor2.setControl(driveMotorRequest.withPosition(position));
    }

    @Override
    public double getPosition() {
        return elevatorPosition;
    }

    @Override
    public double getVelocity() {
        return elevatorVelocity;
    }

    @Override
    public void zeroHeight() {
        elevatorPosition = 0.0;
        elevatorVelocity = 0.0;
    }

}