package frc.robot.commands.autoAligment;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElementConstants;
import frc.robot.constants.swerve.SwerveModuleConfig.GeneralConfig;
import frc.robot.constants.swerve.SwerveCompConfig;
import frc.robot.constants.swerve.SwerveConfigBase;
import frc.robot.constants.swerve.SwerveDrivetrainConfig;
import frc.robot.constants.swerve.SwerveSimConfig;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class AutoAlign extends Command {
    private final SwerveDrive swerveDrive;
    private final SwerveCompConfig config;

    private Pose2d targetPose = ElementConstants.kPOSE_ARR[0];
    private Pose2d currentPose;

    private final TrapezoidProfile translationalMotionProfile;
    private final PIDController xTranslationalFeedbackController;
    private final PIDController yTranslationalFeedbackController;
    private State currentTranslationalGoal = new State();

    private final TrapezoidProfile rotationalMotionProfile;
    private final PIDController rotationalFeedbackController;

    private State currentRotationalSetpoint = new State(0, 0);
    private State currentTranslationalSetpoint = new State(0, 0);

    private double currentUnwrappedRotationRad = 0;
    private double previousWrappedRotationRad = 0;

    double previousTimestamp = Timer.getFPGATimestamp();

    public AutoAlign(SwerveDrive swerveDrive, SwerveCompConfig config) {
        this.swerveDrive = swerveDrive;
        this.config = config;

        this.translationalMotionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC,
                config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_ACCELERATION_METERS_PER_SEC_SEC
            )
        );
        this.xTranslationalFeedbackController = config.getSwerveDrivetrainControllerConfig().kAUTO_ALIGN_TRANSLATIONAL_POSITION_FEEDBACK_CONTROLLER;
        this.yTranslationalFeedbackController = config.getSwerveDrivetrainControllerConfig().kAUTO_ALIGN_TRANSLATIONAL_POSITION_FEEDBACK_CONTROLLER;


        this.rotationalMotionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC,
                config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_ANGULAR_ACCELERATION_RADIANS_PER_SEC_SEC
            )
        );
        this.rotationalFeedbackController = config.getSwerveDrivetrainControllerConfig().kAUTO_ALIGN_ROTATIONAL_POSITION_FEEDBACK_CONTROLLER;

        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize() {
        double distanceToPose;
        currentPose = swerveDrive.getPose();

        for (int i = 0; i < ElementConstants.kPOSE_ARR.length; i++) {
            distanceToPose = calculateDistance(currentPose, ElementConstants.kPOSE_ARR[i]);
            if (distanceToPose < calculateDistance(currentPose, targetPose)) {
                targetPose = ElementConstants.kPOSE_ARR[i];
            }
        }
        
        currentTranslationalSetpoint = new State(
            calculateDistance(currentPose, targetPose),
            Math.hypot(
                swerveDrive.getMeasuredFieldRelativeSpeeds().vxMetersPerSecond, 
                swerveDrive.getMeasuredFieldRelativeSpeeds().vyMetersPerSecond)
        );

        currentTranslationalGoal = new State(
            calculateDistance(currentPose, targetPose),
            0
        );

        currentRotationalSetpoint = new State(
            targetPose.getRotation().getRadians(),
            swerveDrive.getMeasuredFieldRelativeSpeeds().omegaRadiansPerSecond
        );

        Logger.recordOutput("AutoAlign/targetPose", targetPose);
    }

    @Override
    public void execute() {
        Logger.recordOutput("AutoAlign/currentMaxVelo", config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC);
        Logger.recordOutput("AutoAlign/currentMaxAccel", config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_ACCELERATION_METERS_PER_SEC_SEC);
        Logger.recordOutput("AutoAlign/currentMaxAngVelo", config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC);
        Logger.recordOutput("AutoAlign/currentMaxAngAccel", config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_ANGULAR_ACCELERATION_RADIANS_PER_SEC_SEC);

        double dt = Timer.getFPGATimestamp() - previousTimestamp;
        previousTimestamp = Timer.getFPGATimestamp();

        // get the setpoints from the motion profiles
        currentTranslationalSetpoint = translationalMotionProfile.calculate(
                    dt,
                    currentTranslationalSetpoint,
                    currentTranslationalGoal
        );

        double rotationDelta = swerveDrive.getPose().getRotation().getRadians() - previousWrappedRotationRad;
        rotationDelta = MathUtil.inputModulus(rotationDelta, -Math.PI, Math.PI);
        
        previousWrappedRotationRad = swerveDrive.getPose().getRotation().getRadians();
        currentUnwrappedRotationRad += rotationDelta;

        double desiredRotationRad = MathUtil.angleModulus(targetPose.getRotation().getRadians());
        double rotationError = desiredRotationRad - currentUnwrappedRotationRad;
        rotationError = MathUtil.inputModulus(rotationError, -Math.PI, Math.PI);

        currentRotationalSetpoint = rotationalMotionProfile.calculate(
                    dt,
                    currentRotationalSetpoint,
                    new State(
                        currentUnwrappedRotationRad + rotationError,
                            0
                    )
        );

        // calculate the outputs
        double vx, vy, omega;
        double angle = calculateAngle(swerveDrive.getPose(), targetPose);

        // xTranslationalFeedbackController.setSetpoint(targetPose.getTranslation().getX());
        vx = currentTranslationalSetpoint.velocity * Math.cos(angle) + xTranslationalFeedbackController.calculate(swerveDrive.getPose().getTranslation().getX(),
                                                                                                                  targetPose.getTranslation().getX());
        
        // yTranslationalFeedbackController.setSetpoint(targetPose.getTranslation().getY());
        vy = currentTranslationalSetpoint.velocity * Math.sin(angle) + yTranslationalFeedbackController.calculate(swerveDrive.getPose().getTranslation().getY(),
                                                                                                                  targetPose.getTranslation().getY());                    

        // rotationalFeedbackController.setSetpoint(targetPose.getRotation().getRadians());
        omega = currentRotationalSetpoint.velocity + rotationalFeedbackController.calculate(swerveDrive.getPose().getRotation().getRadians(),
                                                                                            targetPose.getRotation().getRadians());

        // set the outputs
        swerveDrive.driveFieldRelative(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            swerveDrive.driveFieldRelative(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        return xTranslationalFeedbackController.atSetpoint() && yTranslationalFeedbackController.atSetpoint() && rotationalFeedbackController.atSetpoint();
    }    

    private static double calculateDistance(Pose2d currentPose, Pose2d targetPose) {
        return currentPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    private static double calculateAngle(Pose2d currentPose, Pose2d targetPose) {
        return Math.atan2(currentPose.relativeTo(targetPose).getY(), currentPose.relativeTo(targetPose).getX());
    }
}