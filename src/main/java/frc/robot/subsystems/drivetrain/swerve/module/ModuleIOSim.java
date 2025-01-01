package frc.robot.subsystems.drivetrain.swerve.module;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.SwerveModuleConfigs.SwerveModuleConfig;
import frc.robot.lib.util.RebelUtil;

public class ModuleIOSim implements ModuleIO {
    private DCMotor gearBoxSteer = DCMotor.getKrakenX60Foc(1);
    private DCMotor gearBoxDrive = DCMotor.getKrakenX60Foc(1);

    private FlywheelSim steerSim;
    private FlywheelSim driveSim;
    
    private final PIDController steerFeedback;
    private final PIDController driveFeedback;

    private final SimpleMotorFeedforward steerFeedforward;
    private final SimpleMotorFeedforward driveFeedforward;

    private double prevTimeInputs = 0;
    private double prevTimeState = 0;


    // TODO: These shouold be moved into a general swerve config and calculated bassed off mass and drivebase (pathplanner)
    private final double kDRIVE_JKG_METERS_SQUARED = 0.0001;
    private final double kSTEER_JKG_METERS_SQUARED = 0.0001;

    private final double kDRIVE_MOTOR_ROTATIONS_TO_METERS; 
    private final double kDRIVE_METERS_TO_MOTOR_ROTATIONS; 

    private final double kSTEER_MOTOR_ROTATIONS_TO_MODULE_ROTATIONS;
    private final double kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS;

    private final SlewRateLimiter driveSlewRateLimiter;
    private final TrapezoidProfile steerMotionProfile;

    private final SwerveModuleConfig config;

    private State currentSteerState = new State(0,0);
    private State currentDriveState = new State(0,0);

    public ModuleIOSim(SwerveModuleConfig config) {
        this.config = config;

        // TODO: CHECK THIS !!!!!!!!!!!!!
        kDRIVE_MOTOR_ROTATIONS_TO_METERS =
            config.kGENERAL_CONFIG.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO * 
            2 * Math.PI * config.kGENERAL_CONFIG.kDRIVE_WHEEL_RADIUS_METERS;

        kDRIVE_METERS_TO_MOTOR_ROTATIONS = 1 / kDRIVE_MOTOR_ROTATIONS_TO_METERS;

        kSTEER_MOTOR_ROTATIONS_TO_MODULE_ROTATIONS = config.kGENERAL_CONFIG.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;
        kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS =  1 / config.kGENERAL_CONFIG.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;

        driveSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                gearBoxDrive, 
                kDRIVE_JKG_METERS_SQUARED, 
                1/config.kGENERAL_CONFIG.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO// TODO: CHECK!!!!
            ),
            gearBoxSteer,
            .23
        );

        steerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                gearBoxSteer, 
                kSTEER_JKG_METERS_SQUARED, 
                kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS // TODO: CHECK!!!!
            ),
            gearBoxSteer,
            .23
        );
        
        driveFeedback = new PIDController(
            config.kGENERAL_CONFIG.kDRIVE_KP,
            config.kGENERAL_CONFIG.kDRIVE_KI,
            config.kGENERAL_CONFIG.kDRIVE_KD
        );
        steerFeedback = new PIDController(
            config.kGENERAL_CONFIG.kSTEER_KP,
            config.kGENERAL_CONFIG.kSTEER_KI,
            config.kGENERAL_CONFIG.kSTEER_KD
        );

        driveFeedforward = new SimpleMotorFeedforward(
            config.kGENERAL_CONFIG.kDRIVE_KS,
            config.kGENERAL_CONFIG.kDRIVE_KV,
            config.kGENERAL_CONFIG.kDRIVE_KA
        );
        steerFeedforward = new SimpleMotorFeedforward(
            config.kGENERAL_CONFIG.kSTEER_KS,
            config.kGENERAL_CONFIG.kSTEER_KV,
            config.kGENERAL_CONFIG.kSTEER_KA
        );

        driveFeedback.setTolerance(0.05);
        steerFeedback.setTolerance(Math.toRadians(1));

        steerFeedback.enableContinuousInput(0, 2 * Math.PI);
        // TOOD: because the rotation of a Rotation2d is automatically 0 to 2pi, this is not needed
        // // unsigned sensor
        // if (config.kGENERAL_CONFIG.kCANCODER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT == 1) {
        //     steerFeedback.enableContinuousInput(0, 2 * Math.PI);
        // }
        // // signed sensor (0.5)
        // else {
        //     steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
        // }
        
        driveSlewRateLimiter = new SlewRateLimiter(config.kGENERAL_CONFIG.kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC);

        steerMotionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            config.kGENERAL_CONFIG.kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC,
            12/config.kGENERAL_CONFIG.kSTEER_MOTION_MAGIC_EXPO_KA // divide supply voltage to get max acell
        ));

    }
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double dt = Timer.getFPGATimestamp() - prevTimeInputs;

        steerSim.update(dt);
        driveSim.update(dt);

        inputs.driveVelocityMetersPerSec = 
            driveSim.getAngularVelocityRPM() * 
            2 * Math.PI * config.kGENERAL_CONFIG.kDRIVE_WHEEL_RADIUS_METERS;
        inputs.drivePositionMeters += inputs.driveVelocityMetersPerSec * dt;

        inputs.driveCurrentDrawAmps = driveSim.getCurrentDrawAmps();
        inputs.driveAppliedVolts = driveSim.getInputVoltage();
        inputs.driveTemperatureFahrenheit = 60; // random number

        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerCANCODERAbsolutePosition = new Rotation2d(
            inputs.steerCANCODERAbsolutePosition.getRadians() +
            inputs.steerVelocityRadPerSec * dt
        );
        
        inputs.steerPosition = inputs.steerCANCODERAbsolutePosition;

        inputs.steerCurrentDrawAmps = steerSim.getCurrentDrawAmps();
        inputs.steerAppliedVolts = steerSim.getInputVoltage();
        inputs.steerTemperatureFahrenheit = 60; // random number

        currentSteerState = new State(inputs.steerPosition.getRadians(), inputs.steerVelocityRadPerSec);
        currentDriveState = new State(inputs.drivePositionMeters, inputs.driveVelocityMetersPerSec);

        prevTimeInputs = Timer.getFPGATimestamp();
    }

    @Override
    public void setState(SwerveModuleState state) {
        double dt = Timer.getFPGATimestamp() - prevTimeState;
        // a setpoint is the individual setpoint for the motor calculated by the profile
        // while the goal is the end state
        double driveSetpointMetersPerSec = RebelUtil.constrain(
            driveSlewRateLimiter.calculate(state.speedMetersPerSecond),
            -config.kGENERAL_CONFIG.kDRIVE_MAX_VELOCITY_METERS_PER_SEC,
            config.kGENERAL_CONFIG.kDRIVE_MAX_VELOCITY_METERS_PER_SEC
        );

        State steerSetpoint = 
            steerMotionProfile.calculate(
                dt, 
                currentSteerState, 
                new State(
                    state.angle.getRadians(), 
                    0
                )
            );
        
        double driveInputVoltage = 
            driveFeedback.calculate(
                currentDriveState.velocity, 
                driveSetpointMetersPerSec
            ) + 
            driveFeedforward.calculate(driveSetpointMetersPerSec);
        
        double steerInputVoltage = 
            steerFeedback.calculate(
                currentSteerState.position, 
                steerSetpoint.position
            ) + 
            steerFeedforward.calculate(steerSetpoint.velocity);
        
        driveSim.setInputVoltage(driveInputVoltage);
        steerSim.setInputVoltage(steerInputVoltage);

        prevTimeState = Timer.getFPGATimestamp();
    }

    @Override
    public void setDriveVoltage(double driveInputVoltage) {
        driveSim.setInputVoltage(driveInputVoltage);

    }
}
