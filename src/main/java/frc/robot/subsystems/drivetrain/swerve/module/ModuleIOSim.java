package frc.robot.subsystems.drivetrain.swerve.module;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.SwerveConfigBase;
import frc.robot.constants.SwerveModuleConfig.GeneralConfig;
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

    private final SwerveConfigBase configBase;
    private final GeneralConfig generalConfig;

    private State currentSteerState = new State(0,0);
    private State currentDriveState = new State(0,0);

    @SuppressWarnings("static-access")
    public ModuleIOSim(SwerveConfigBase configBase) {
        this.configBase = configBase;
        this.generalConfig = configBase.kSHARED_GENERAL_CONFIG;

        // TODO: CHECK THIS !!!!!!!!!!!!!
        kDRIVE_MOTOR_ROTATIONS_TO_METERS =
            generalConfig.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO * 
            2 * Math.PI * generalConfig.kDRIVE_WHEEL_RADIUS_METERS;

        kDRIVE_METERS_TO_MOTOR_ROTATIONS = 1 / kDRIVE_MOTOR_ROTATIONS_TO_METERS;

        kSTEER_MOTOR_ROTATIONS_TO_MODULE_ROTATIONS = generalConfig.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;
        kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS =  1 / generalConfig.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;

        driveSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                gearBoxDrive, 
                kDRIVE_JKG_METERS_SQUARED, 
                1/generalConfig.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO// TODO: CHECK!!!!
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
            generalConfig.kDRIVE_KP,
            generalConfig.kDRIVE_KI,
            generalConfig.kDRIVE_KD
        );
        steerFeedback = new PIDController(
            generalConfig.kSTEER_KP,
            generalConfig.kSTEER_KI,
            generalConfig.kSTEER_KD
        );

        driveFeedforward = new SimpleMotorFeedforward(
            generalConfig.kDRIVE_KS,
            generalConfig.kDRIVE_KV,
            generalConfig.kDRIVE_KA
        );
        steerFeedforward = new SimpleMotorFeedforward(
            generalConfig.kSTEER_KS,
            generalConfig.kSTEER_KV,
            generalConfig.kSTEER_KA
        );

        driveFeedback.setTolerance(0.05);
        steerFeedback.setTolerance(Math.toRadians(1));

        steerFeedback.enableContinuousInput(0, 2 * Math.PI);
        // TOOD: because the rotation of a Rotation2d is automatically 0 to 2pi, this is not needed
        // // unsigned sensor
        // if (generalConfig.kCANCODER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT == 1) {
        //     steerFeedback.enableContinuousInput(0, 2 * Math.PI);
        // }
        // // signed sensor (0.5)
        // else {
        //     steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
        // }
        
        driveSlewRateLimiter = new SlewRateLimiter(generalConfig.kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC);

        steerMotionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            generalConfig.kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC,
            12/generalConfig.kSTEER_MOTION_MAGIC_EXPO_KA // divide supply voltage to get max acell
        ));

    }
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double dt = Timer.getFPGATimestamp() - prevTimeInputs;

        steerSim.update(dt);
        driveSim.update(dt);

        inputs.timestamp = HALUtil.getFPGATime() / 1.0e6;

        inputs.driveVelocityMetersPerSec = 
            driveSim.getAngularVelocityRPM() * 
            2 * Math.PI * generalConfig.kDRIVE_WHEEL_RADIUS_METERS;
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
            -generalConfig.kDRIVE_MAX_VELOCITY_METERS_PER_SEC,
            generalConfig.kDRIVE_MAX_VELOCITY_METERS_PER_SEC
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
