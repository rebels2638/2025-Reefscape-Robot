package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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

public class ClimberIOSim implements ClimberIO {
    private DCMotor climberGearBox = DCMotor.getKrakenX60Foc(1);

    private SingleJointedArmSim climberSim;

    private final PIDController feedbackController;
    private final ArmFeedforward feedforwardController;

    private double prevTimeInputs = 0;
    private double prevTimeState = 0;

    private final double kclimber_MOTOR_TO_OUTPUT_SHAFT_RATIO = 10;
    private final double kJKG_METERS_SQUARED = 0.1;
    private final double kclimber_LENGTH_METERS = 0.222;
    private final double kMIN_ANGLE_RAD;
    private final double kMAX_ANGLE_RAD;
    private final double kSTARTING_ANGLE_RAD;

    private final TrapezoidProfile trapezoidMotionProfile;
    private State currentProfileSetpoint;

    private double previousAngleRad = 0;
    private double appliedVolts = 0;
    
    public ClimberIOSim(ClimberConfigBase config) {
        kMIN_ANGLE_RAD = config.getMinAngleRotations() * Math.PI * 2;
        kMAX_ANGLE_RAD = config.getMaxAngleRotations() * Math.PI * 2;
        kSTARTING_ANGLE_RAD = Math.toRadians(0);

        Logger.recordOutput("climber/minAngleRad", kMIN_ANGLE_RAD);
        Logger.recordOutput("climber/maxAngleRad", kMAX_ANGLE_RAD);

        climberSim = new SingleJointedArmSim(
            climberGearBox,
            kclimber_MOTOR_TO_OUTPUT_SHAFT_RATIO,
            kJKG_METERS_SQUARED,
            kclimber_LENGTH_METERS,
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

        feedbackController.enableContinuousInput(-Math.PI, Math.PI);

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

        currentProfileSetpoint = new State(kSTARTING_ANGLE_RAD, 0);
        previousAngleRad = MathUtil.angleModulus(kSTARTING_ANGLE_RAD);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInputs;
        prevTimeInputs = Timer.getTimestamp();

        climberSim.update(dt);

        inputs.climberVelocityRadPerSec = climberSim.getVelocityRadPerSec();
        inputs.climberPosition = new Rotation2d(MathUtil.angleModulus(climberSim.getAngleRads()));
        previousAngleRad = inputs.climberPosition.getRadians();

        inputs.climberAppliedVolts = appliedVolts;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        double dt = Timer.getTimestamp() - prevTimeState;
        prevTimeState = Timer.getTimestamp();

        currentProfileSetpoint = trapezoidMotionProfile.calculate(
            dt, 
            currentProfileSetpoint,
            new State(
                angle.getRadians(),
                0
            )
        );
        Logger.recordOutput("climber/currentProfileSetpoint/position", currentProfileSetpoint.position);

        double voltage = 
            feedforwardController.calculate(currentProfileSetpoint.position, currentProfileSetpoint.velocity) +
            feedbackController.calculate(previousAngleRad, currentProfileSetpoint.position);
        appliedVolts = voltage;

        climberSim.setInputVoltage(voltage);
    }

    @Override
    public void setTorqueCurrentFOC(double voltage) {
        appliedVolts = voltage;
        climberSim.setInputVoltage(voltage);

    }
}
