package frc.robot.subsystems.drivetrain.swerve.module;

import java.io.Console;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.moduleConfigs.SwerveModuleGeneralConfigBase;
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

    // TODO: These shouold be moved into a general swerve config and calculated
    // bassed off mass and drivebase (pathplanner)
    private final double kDRIVE_JKG_METERS_SQUARED = 0.025;
    private final double kSTEER_JKG_METERS_SQUARED = 0.0008096955;

    private final double kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS;

    private final SlewRateLimiter driveSlewRateLimiter;
    private final TrapezoidProfile steerMotionProfile;

    private final SwerveModuleGeneralConfigBase config;
    private final int moduleID;

    private State currentSteerState = new State(0, 0);
    private State currentDriveState = new State(0, 0);

    private State currentSteerSetpoint = new State(0, 0);
    private double currentUnwrappedAngleRad = 0;
    private double previousWrappedAngleRad = 0;

    double previousDriveDesiredVeloMps = 0;

    double previousNonZeroDriveSetpoint = 1;

    public ModuleIOSim(SwerveModuleGeneralConfigBase config, int moduleID) {
        this.config = config;
        this.moduleID = moduleID;

        kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS = 1 / config.getSteerMotorToOutputShaftRatio();

        driveSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        gearBoxDrive,
                        kDRIVE_JKG_METERS_SQUARED,
                        1 / config.getDriveMotorToOutputShaftRatio()// TODO: CHECK!!!!
                ),
                gearBoxSteer);

        steerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        gearBoxSteer,
                        kSTEER_JKG_METERS_SQUARED,
                        kSTEER_MODULE_ROTATIONS_TO_MOTOR_ROTATIONS // TODO: CHECK!!!!
                ),
                gearBoxSteer);

        driveFeedback = new PIDController(
                config.getDriveKP(),
                config.getDriveKI(),
                config.getDriveKD());
        steerFeedback = new PIDController(
                config.getSteerKP(),
                config.getSteerKI(),
                config.getSteerKD());

        driveFeedforward = new SimpleMotorFeedforward(
                config.getDriveKS(),
                config.getDriveKV(),
                config.getDriveKA());
        steerFeedforward = new SimpleMotorFeedforward(
                config.getSteerKS(),
                config.getSteerKV(),
                config.getSteerKA());

        driveFeedback.setTolerance(0.05);
        steerFeedback.setTolerance(Math.toRadians(1));

        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);

        driveSlewRateLimiter = new SlewRateLimiter(
                config.getDriveMotionMagicVelocityAccelerationMetersPerSecSec(),
                -config.getDriveMotionMagicVelocityDecelerationMetersPerSecSec(),
                0);

        steerMotionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                Units.rotationsToRadians(config.getSteerMotionMagicCruiseVelocityRotationsPerSec()),
                12 / config.getSteerMotionMagicExpoKA() // divide supply voltage to get max acell
        ));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInputs;
        prevTimeInputs = Timer.getTimestamp();

        steerSim.update(dt);
        driveSim.update(dt);

        inputs.timestamp = Timer.getTimestamp();

        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRPM() *
                2 * Math.PI * config.getDriveWheelRadiusMeters();
        inputs.driveAccelerationMetersPerSecSec = driveSim.getAngularAccelerationRadPerSecSq() * config.getDriveWheelRadiusMeters();
        inputs.drivePositionMeters += inputs.driveVelocityMetersPerSec * dt;

        inputs.driveCurrentDrawAmps = driveSim.getCurrentDrawAmps();
        inputs.driveAppliedVolts = driveSim.getInputVoltage();
        inputs.driveTemperatureFahrenheit = 60; // random number

        inputs.steerPosition = new Rotation2d(
            MathUtil.angleModulus(
                inputs.steerPosition.getRadians() +
                inputs.steerVelocityRadPerSec * dt
            )
        );
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();

        double angleDelta = inputs.steerPosition.getRadians() - previousWrappedAngleRad;
        previousWrappedAngleRad = inputs.steerPosition.getRadians();

        angleDelta = MathUtil.inputModulus(angleDelta, -Math.PI, Math.PI);

        // Accumulate into continuous angle
        currentUnwrappedAngleRad += angleDelta;

        inputs.steerCurrentDrawAmps = steerSim.getCurrentDrawAmps();
        inputs.steerAppliedVolts = steerSim.getInputVoltage();
        inputs.steerTemperatureFahrenheit = 60; // random number

        currentSteerState = new State(inputs.steerPosition.getRadians(), inputs.steerVelocityRadPerSec);
        currentDriveState = new State(inputs.drivePositionMeters, inputs.driveVelocityMetersPerSec);
        Logger.recordOutput("SwerveDrive/module" + moduleID + "/currentSteerStateRad",
                inputs.steerPosition.getRadians());

    }

    @Override
    public void setState(SwerveModuleState state) {
        double dt = Timer.getTimestamp() - prevTimeState;
        prevTimeState = Timer.getTimestamp();

        // a setpoint is the individual setpoint for the motor calculated by the profile
        // while the goal is the end state
        double driveSetpointMetersPerSec = RebelUtil.constrain(
                state.speedMetersPerSecond,
                -config.getDriveMaxVelocityMetersPerSec(),
                config.getDriveMaxVelocityMetersPerSec());

        double driveAccel = (driveSetpointMetersPerSec - previousDriveDesiredVeloMps) / dt;
        Logger.recordOutput("SwerveDrive/module" + moduleID + "/driveAccel", driveAccel);

        if (driveAccel >= 0 && previousDriveDesiredVeloMps >= 0) {
            driveAccel = Math.signum(driveAccel) * Math.min(Math.abs(driveAccel), config.getDriveMotionMagicVelocityAccelerationMetersPerSecSec());
        } 
        else if (driveAccel <= 0 && previousDriveDesiredVeloMps <= 0) {
            driveAccel = Math.signum(driveAccel) * Math.min(Math.abs(driveAccel), config.getDriveMotionMagicVelocityAccelerationMetersPerSecSec());
        }
        else if (driveAccel >= 0 && previousDriveDesiredVeloMps <= 0) {
            driveAccel = Math.signum(driveAccel) * Math.min(Math.abs(driveAccel), config.getDriveMotionMagicVelocityDecelerationMetersPerSecSec());
        }
        else if (driveAccel <= 0 && previousDriveDesiredVeloMps >= 0) {
            driveAccel = Math.signum(driveAccel) * Math.min(Math.abs(driveAccel), config.getDriveMotionMagicVelocityDecelerationMetersPerSecSec());
        }
        else {
            driveAccel = 0;
        }

        driveSetpointMetersPerSec = previousDriveDesiredVeloMps + driveAccel * dt;
        previousDriveDesiredVeloMps = driveSetpointMetersPerSec;

        Logger.recordOutput("SwerveDrive/module" + moduleID + "/driveSetpointMetersPerSec", driveSetpointMetersPerSec);

        double desiredAngleRad = MathUtil.angleModulus(state.angle.getRadians());
        double angleError = desiredAngleRad - currentUnwrappedAngleRad;
        angleError = MathUtil.inputModulus(angleError, -Math.PI, Math.PI);

        Logger.recordOutput("SwerveDrive/module" + moduleID + "/angleError", angleError);        
        Logger.recordOutput("SwerveDrive/module" + moduleID + "/desiredAngleRad", desiredAngleRad);

        currentSteerSetpoint = steerMotionProfile.calculate(
                    dt,
                    currentSteerSetpoint,
                    new State(
                        currentUnwrappedAngleRad + angleError,
                            0));
        State nextSteerSetpoint = steerMotionProfile.calculate(
                    dt * 2,
                    currentSteerSetpoint,
                    new State(
                        currentUnwrappedAngleRad + angleError,
                            0));
        Logger.recordOutput("SwerveDrive/module" + moduleID + "/currentSteerSetpointVelocity", currentSteerSetpoint.velocity);
        Logger.recordOutput("SwerveDrive/module" + moduleID + "/currentSteerSetpointPosition", currentSteerSetpoint.position);
        Logger.recordOutput("SwerveDrive/module" + moduleID + "/angleError", angleError);
        double driveInputVoltage = driveFeedback.calculate(
                currentDriveState.velocity,
                driveSetpointMetersPerSec) +
                driveFeedforward.calculate(driveSetpointMetersPerSec);

        Logger.recordOutput("SwerveDrive/module" + moduleID + "/currentSteerSetpointAngleModulus", MathUtil.angleModulus(currentSteerSetpoint.position));

        double steerInputVoltage = steerFeedback.calculate(
                MathUtil.angleModulus(currentSteerState.position),
                MathUtil.angleModulus(currentSteerSetpoint.position)) +
                steerFeedforward.calculate(
                        currentSteerSetpoint.velocity,
                        (nextSteerSetpoint.velocity - currentSteerSetpoint.velocity) / dt
                       );

        driveSim.setInputVoltage(driveInputVoltage);
        steerSim.setInputVoltage(steerInputVoltage);
    }

    @Override
    public void setDriveVoltage(double driveInputVoltage) {
        driveSim.setInputVoltage(driveInputVoltage);

    }
}
