package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.climber.ClimberConfigBase;
import frc.robot.constants.pivot.PivotConfigBase;

public class ClimberIOSim implements ClimberIO {
    private DCMotor pivotGearBox = DCMotor.getKrakenX60Foc(1);

    private SingleJointedArmSim pivotSim;

    private final PIDController feedbackController;
    private final ArmFeedforward feedforwardController;

    private double prevTimeInputs = 0;
    private double prevTimeState = 0;

    private final double kPIVOT_MOTOR_TO_OUTPUT_SHAFT_RATIO = 20;
    private final double kJKG_METERS_SQUARED = 11.34;
    private final double kPIVOT_LENGTH_METERS = 0.23;
    private final double kMIN_ANGLE_RAD;
    private final double kMAX_ANGLE_RAD;
    private final double kSTARTING_ANGLE_RAD = Math.toRadians(0);


    private final TrapezoidProfile trapezoidMotionProfile;
    private State currentProfileSetpoint;;

    private double currentPositionRad = 0;
    private double appliedVolts = 0;
    
    public ClimberIOSim(ClimberConfigBase config) {
        kMIN_ANGLE_RAD = config.getMinAngleRotations() * Math.PI * 2;
        kMAX_ANGLE_RAD = config.getMaxAngleRotations() * Math.PI * 2;
        
        Logger.recordOutput("Pivot/minAngleRad", kMIN_ANGLE_RAD);
        Logger.recordOutput("Pivot/maxAngleRad", kMAX_ANGLE_RAD);

        pivotSim = new SingleJointedArmSim(
            pivotGearBox,
            kPIVOT_MOTOR_TO_OUTPUT_SHAFT_RATIO,
            kJKG_METERS_SQUARED,
            kPIVOT_LENGTH_METERS,
            kMIN_ANGLE_RAD,
            kMAX_ANGLE_RAD,
            true,
            kSTARTING_ANGLE_RAD
        );

        feedbackController = new PIDController(
            config.getKP(), 
            config.getKI(), 
            config.getKD()
        );

        feedforwardController = new ArmFeedforward(
            config.getKS(), 
            config.getKG(), 
            config.getKV(),
            config.getKA()
        );

        trapezoidMotionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Units.rotationsToRadians(config.getMotionMagicCruiseVelocityRotationsPerSec()),
                12 / config.getMotionMagicExpoKA() // divide supply voltage to get max acell
        ));
        currentProfileSetpoint = new State(pivotSim.getAngleRads(), pivotSim.getVelocityRadPerSec());
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInputs;
        pivotSim.update(dt);

        inputs.climberVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
        inputs.climberPosition = new Rotation2d(pivotSim.getAngleRads());
        this.currentPositionRad = inputs.climberPosition.getRadians();

        inputs.climberAppliedVolts = appliedVolts;

        prevTimeInputs = Timer.getTimestamp();
    }

    @Override
    public void setAngle(Rotation2d angle) {
        double dt = Timer.getTimestamp() - prevTimeState;
        
        currentProfileSetpoint = trapezoidMotionProfile.calculate(
            dt, 
            currentProfileSetpoint,
            new State(
                angle.getRadians(),
                0
            )
        );

        double voltage = 
            feedforwardController.calculate(currentProfileSetpoint.position, currentProfileSetpoint.velocity) +
            feedbackController.calculate(currentPositionRad, currentProfileSetpoint.position);
        appliedVolts = voltage;

        pivotSim.setInputVoltage(voltage);

        prevTimeState = Timer.getTimestamp();
    }

    @Override
    public void setTorqueCurrentFOC(double voltage) {
        appliedVolts = voltage;
        pivotSim.setInputVoltage(voltage);

    }
}
