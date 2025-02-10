package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.constants.pivot.PivotConfigBase;

public class ElevatorIOSim implements ElevatorIO {
    private DCMotor elevatorGearBox = DCMotor.getKrakenX60Foc(2);

    private ElevatorSim elevatorSim;

    private final PIDController feedbackController;
    private final ElevatorFeedforward feedforwardController;

    private double prevTimeInputs = 0;
    private double prevTimeState = 0;

    private final double kELEVATOR_MOTOR_TO_OUTPUT_SHAFT_RATIO = 20;
    private final double kJKG_METERS_SQUARED = 0.0008096955;
    private final double kELEVATOR_LENGTH_METERS = 0.7;
    private final double kMIN_HEIGHT_RAD = 0.7;
    private final double kMAX_HEIGHT_RAD = 1.4;
    private final double kSTARTING_HEIGHT_RAD = 0.7;

    private final TrapezoidProfile trapezoidMotionProfile;
    private State currentProfileSetpoint;

    private double currentPositionRad = 0;
    private double appliedVolts = 0;
    
    @SuppressWarnings("static-access")
    public ElevatorIOSim(ElevatorConfigBase config) {
        elevatorSim = new ElevatorSim(
            elevatorGearBox,
            kELEVATOR_MOTOR_TO_OUTPUT_SHAFT_RATIO,
            kJKG_METERS_SQUARED,
            kELEVATOR_LENGTH_METERS,
            kMIN_HEIGHT_RAD,
            kMAX_HEIGHT_RAD,
            true,
            kSTARTING_HEIGHT_RAD
        );

        feedbackController = new PIDController(config.getKP(), config.getKI(), config.getKD());
        feedforwardController = new ElevatorFeedforward(config.getKS(), config.getKG(), config.getKV());

        feedbackController.setTolerance(0.03);

        trapezoidMotionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                config.getMotionMagicCruiseVelocityMetersPerSec(),
                12 / config.getMotionMagicExpoKA() // divide supply voltage to get max acell
        ));
        currentProfileSetpoint = new State(elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInputs;
        elevatorSim.update(dt);

        inputs.elevatorVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.elevatorHeightMeters = elevatorSim.getPositionMeters();
        this.currentPositionRad = inputs.elevatorHeightMeters;

        inputs.elevatorAppliedVolts1 = appliedVolts;
        inputs.elevatorAppliedVolts2 = appliedVolts;

        prevTimeInputs = Timer.getTimestamp();
    }

    @Override
    public void setHeight(double height) {
        double dt = Timer.getTimestamp() - prevTimeState;
        
        currentProfileSetpoint = trapezoidMotionProfile.calculate(
            dt, 
            currentProfileSetpoint,
            new State(
                height,
                0
            )
        );

        double voltage = 
            feedforwardController.calculate(currentProfileSetpoint.position, currentProfileSetpoint.velocity) +
            feedbackController.calculate(currentPositionRad, currentProfileSetpoint.position);
        appliedVolts = voltage;

        elevatorSim.setInputVoltage(voltage);

        prevTimeState = Timer.getTimestamp();
    }

    @Override
    public void setTorqueCurrentFOC(double voltage) {
        appliedVolts = voltage;
        elevatorSim.setInputVoltage(voltage);

    }
}
