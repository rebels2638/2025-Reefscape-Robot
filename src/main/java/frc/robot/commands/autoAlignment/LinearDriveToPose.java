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

    private final TrapezoidProfile xTranslationalMotionProfile;
    private final PIDController xTranslationalFeedbackController;

    private State xTranslationalGoal = new State();
    private State currentXTranslationalSetpoint = new State();

    private final TrapezoidProfile yTranslationalMotionProfile;
    private final PIDController yTranslationalFeedbackController;

    private State yTranslationalGoal = new State();
    private State currentYTranslationalSetpoint = new State();

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

        this.xTranslationalMotionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                drivetrainConfig.getMaxDrivetrainTranslationalVelocityMetersPerSec(),
                drivetrainConfig.getMaxDrivetrainTranslationalAccelerationMetersPerSecSec()
            )
        );

        this.yTranslationalMotionProfile = new TrapezoidProfile(
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
        System.out.println("init");

        this.xTranslationalGoal = new State(
            targetPose.get().getX(),
            endVelo.get().vxMetersPerSecond
        );

        this.yTranslationalGoal = new State(
            targetPose.get().getY(),
            endVelo.get().vyMetersPerSecond
        );

        // rotation wrapping
        double desiredRotationRad = MathUtil.angleModulus(targetPose.get().getRotation().getRadians());
        double rotationError = MathUtil.inputModulus(
            desiredRotationRad - robotState.getEstimatedPose().getRotation().getRadians(),
            -Math.PI, Math.PI
        );

        this.rotationalGoal = new State(
            robotState.getEstimatedPose().getRotation().getRadians() + rotationError,
            endVelo.get().omegaRadiansPerSecond
        );
        

        currentXTranslationalSetpoint = new State(
            robotState.getEstimatedPose().getX(),
            robotState.getFieldRelativeSpeeds().vxMetersPerSecond
        );

        currentYTranslationalSetpoint = new State(
            robotState.getEstimatedPose().getY(),
            robotState.getFieldRelativeSpeeds().vyMetersPerSecond
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

        currentXTranslationalSetpoint = xTranslationalMotionProfile.calculate(
            dt, 
            currentXTranslationalSetpoint, 
            xTranslationalGoal
        );
        Logger.recordOutput("LinearDriveToPose/currentXTranslationalSetpointVelo", currentXTranslationalSetpoint.velocity);

        currentYTranslationalSetpoint = yTranslationalMotionProfile.calculate(
            dt, 
            currentYTranslationalSetpoint, 
            yTranslationalGoal
        );
        Logger.recordOutput("LinearDriveToPose/currentYTranslationalSetpointVelo", currentYTranslationalSetpoint.velocity);
        Logger.recordOutput("LinearDriveToPose/currentYTranslationalSetpointPose", currentYTranslationalSetpoint.position);

        currentRotationalSetpoint = rotationalMotionProfile.calculate(
            dt,
            currentRotationalSetpoint, 
            rotationalGoal
        );

        ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(0, 0, 0);

        calculatedSpeeds.vxMetersPerSecond = 
            xTranslationalFeedbackController.calculate(
                robotState.getEstimatedPose().getX(),
                currentXTranslationalSetpoint.position
            ) + 
            Math.max(
                endVelo.get().vxMetersPerSecond,
                currentXTranslationalSetpoint.velocity
            );

        calculatedSpeeds.vyMetersPerSecond = 
            xTranslationalFeedbackController.calculate(
                robotState.getEstimatedPose().getY(),
                currentYTranslationalSetpoint.position
            ) + 
            Math.max(
                endVelo.get().vyMetersPerSecond,
                currentYTranslationalSetpoint.velocity
            );

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