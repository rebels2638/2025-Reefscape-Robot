package frc.robot.commands.autoAlignment;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class LinearDriveToPose extends Command {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    private final TrapezoidProfile translationalMotionProfile;
    private final PIDController xTranslationalFeedbackController;
    private final PIDController yTranslationalFeedbackController;

    private double translationalMotionProfileRotationRad = 0;
    private Pose2d initialPose = new Pose2d();

    private State translationalGoal = new State();
    private State currentTranslationalSetpoint = new State();

    private final TrapezoidProfile rotationalMotionProfile;
    private final PIDController rotationalFeedbackController;

    private State rotationalGoal = new State(0, 0);
    private State currentRotationalSetpoint = new State(0, 0);

    private double previousTimestamp = Timer.getTimestamp();

    protected Supplier<Pose2d> targetPose;
    protected Supplier<ChassisSpeeds> endVelo;

    private final SwerveDrivetrainConfigBase drivetrainConfig;

    // This is the blue alliance pose! 
    // field relative velo and pose and velocity
    public LinearDriveToPose(Supplier<Pose2d> targetPose, Supplier<ChassisSpeeds> endVelo) {
        switch (Constants.currentMode) {
            case COMP:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();

                break;

            case PROTO:
                drivetrainConfig = SwerveDrivetrainConfigProto.getInstance();
                
                break;
            
            case SIM:
                drivetrainConfig = SwerveDrivetrainConfigSim.getInstance();

                break;

            case REPLAY:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();

                break;

            default:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();

                break;
        }
        
        this.targetPose = targetPose;
        this.endVelo = endVelo;

        this.translationalMotionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                drivetrainConfig.getMaxDrivetrainTranslationalVelocityMetersPerSec(),
                drivetrainConfig.getMaxDrivetrainTranslationalAccelerationMetersPerSecSec()
            )
        );

        this.xTranslationalFeedbackController = drivetrainConfig.getAutoAlignProfiledTranslationController();
        this.yTranslationalFeedbackController = drivetrainConfig.getAutoAlignProfiledTranslationController();

        this.rotationalMotionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                drivetrainConfig.getMaxDrivetrainAngularVelocityRadiansPerSec(),
                drivetrainConfig.getMaxDrivetrainAngularAccelerationRadiansPerSecSec()
            )
        );

        this.rotationalFeedbackController = drivetrainConfig.getAutoAlignProfiledRotationController();

        System.out.println("NEW COMMAND");
        addRequirements(swerveDrive);
    }

    @Override 
    public void initialize() {
        double xDist = targetPose.get().getX() - robotState.getEstimatedPose().getX();
        double yDist = targetPose.get().getY() - robotState.getEstimatedPose().getY();

        initialPose = robotState.getEstimatedPose();

        translationalGoal = new State(
            Math.hypot(xDist, yDist),
            0
        );

        translationalMotionProfileRotationRad = Math.atan2(
            yDist,
            xDist
        );

        Logger.recordOutput("LinearDriveToPose/translationalMotionProfileRotationRad", translationalMotionProfileRotationRad);

        // rotation wrapping
        double desiredRotationRad = MathUtil.angleModulus(targetPose.get().getRotation().getRadians());
        double rotationError = MathUtil.inputModulus(
            desiredRotationRad - robotState.getEstimatedPose().getRotation().getRadians(),
            -Math.PI, Math.PI
        );

        rotationalGoal = new State(
            robotState.getEstimatedPose().getRotation().getRadians() + rotationError,
            endVelo.get().omegaRadiansPerSecond
        );

        currentTranslationalSetpoint = new State(
            0,
            Math.hypot(robotState.getFieldRelativeSpeeds().vxMetersPerSecond, robotState.getFieldRelativeSpeeds().vyMetersPerSecond)
        );

        currentRotationalSetpoint = new State(
            robotState.getEstimatedPose().getRotation().getRadians(),
            robotState.getFieldRelativeSpeeds().omegaRadiansPerSecond
        );

        previousTimestamp = Timer.getTimestamp();

        Logger.recordOutput("LinearDriveToPose/targetPose.get()", targetPose.get());

        xTranslationalFeedbackController.reset();
        yTranslationalFeedbackController.reset();
        rotationalFeedbackController.reset();
    }

    @Override
    public void execute() {
        double dt = Timer.getTimestamp() - previousTimestamp;

        currentTranslationalSetpoint = translationalMotionProfile.calculate(
            dt, 
            currentTranslationalSetpoint, 
            translationalGoal
        );

        Logger.recordOutput("LinearDriveToPose/currentTranslationalSetpointVelo", currentTranslationalSetpoint.velocity);
        Logger.recordOutput("LinearDriveToPose/currentTranslationalSetpointPose", currentTranslationalSetpoint.position);

        currentRotationalSetpoint = rotationalMotionProfile.calculate(
            dt,
            currentRotationalSetpoint, 
            rotationalGoal
        );

        ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(0, 0, 0);

        double xTranslationalSetpoint = currentTranslationalSetpoint.position * Math.cos(translationalMotionProfileRotationRad) + initialPose.getX();
        double xVelocitySetpoint = Math.max(
            endVelo.get().vxMetersPerSecond,
            currentTranslationalSetpoint.velocity * Math.cos(translationalMotionProfileRotationRad)
        );

        calculatedSpeeds.vxMetersPerSecond = 
            xTranslationalFeedbackController.calculate(
                robotState.getEstimatedPose().getX(),
                xTranslationalSetpoint
            ) + xVelocitySetpoint;

        double yTranslationalSetpoint = currentTranslationalSetpoint.position * Math.sin(translationalMotionProfileRotationRad) + initialPose.getY();
        double yVelocitySetpoint = Math.max( // motion profile trips out when nonzero end vel
            endVelo.get().vyMetersPerSecond,
            currentTranslationalSetpoint.velocity * Math.sin(translationalMotionProfileRotationRad)
        );

        calculatedSpeeds.vyMetersPerSecond = 
            xTranslationalFeedbackController.calculate(
                robotState.getEstimatedPose().getY(),
                yTranslationalSetpoint
            ) + yVelocitySetpoint;

        Logger.recordOutput("LinearDriveToPose/xTranslationalSetpoint", xTranslationalSetpoint);
        Logger.recordOutput("LinearDriveToPose/xVelocitySetpoint", xVelocitySetpoint);

        Logger.recordOutput("LinearDriveToPose/yTranslationalSetpoint", yTranslationalSetpoint);
        Logger.recordOutput("LinearDriveToPose/yVelocitySetpoint", yVelocitySetpoint);

        calculatedSpeeds.omegaRadiansPerSecond = 
            rotationalFeedbackController.calculate(
                robotState.getEstimatedPose().getRotation().getRadians(),
                currentRotationalSetpoint.position
            ) + 
            Math.max(
                endVelo.get().omegaRadiansPerSecond,
                currentRotationalSetpoint.velocity
            );

        swerveDrive.driveFieldRelative(calculatedSpeeds);

        previousTimestamp = Timer.getTimestamp();
    }

    @Override
    public boolean isFinished() {
        boolean poseAligned = 
            robotState.getEstimatedPose().getTranslation().getDistance(targetPose.get().getTranslation()) <= 
                drivetrainConfig.getAutoAlignTranslationTolerance() &&
            Math.abs(robotState.getEstimatedPose().getRotation().getRadians() - targetPose.get().getRotation().getRadians()) <= 
                drivetrainConfig.getAutoAlignRotationTolerance();
        
        boolean speedsAligned = 
            endVelo.get().vxMetersPerSecond == 0 && endVelo.get().vyMetersPerSecond == 0 && endVelo.get().omegaRadiansPerSecond == 0 ?
                Math.abs(Math.hypot(
                    robotState.getFieldRelativeSpeeds().vxMetersPerSecond,
                    robotState.getFieldRelativeSpeeds().vyMetersPerSecond
                )) <= drivetrainConfig.getAutoAlignTranslationVeloTolerance() &&
                Math.abs(robotState.getFieldRelativeSpeeds().omegaRadiansPerSecond) <= drivetrainConfig.getAutoAlignRotationVeloTolerance() :
            true;

        return poseAligned && speedsAligned;
    }
}