package frc.robot.commands.autoAlignment;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil.Axis;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;

public class LockDriveAxis extends LinearDriveToPose {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private final SwerveDrivetrainConfigBase drivetrainConfig;

    private final DoubleSupplier vX, vY, heading; // Supplier functions for velocity inputs and heading.
    private int invert = 1;                       // Variable to invert direction based on alliance color.
    
    private final Axis desiredAxis;
    private final Rotation2d desiredRotation;

    private double previousTimestamp = Timer.getTimestamp();
    
    public LockDriveAxis(XboxController xboxDriver, Axis desiredAxis, Rotation2d desiredRotation) {
        super(() -> RobotState.getInstance().getEstimatedPose(), () -> new ChassisSpeeds());
        
        this.desiredAxis = desiredAxis;
        this.desiredRotation = desiredRotation;

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

        this.vX = () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND);
        this.vY = () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND);
        this.heading = () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        super.initialize();

        boolean isRed = false;
        Optional<Alliance> alliance = DriverStation.getAlliance(); // Get the current alliance.

        // Check if the alliance is Red and set invert accordingly.
        if (alliance.isPresent()) {
            isRed = alliance.get() == DriverStation.Alliance.Red;
        }

        // Invert drive direction if the robot is on the Red alliance.
        if (isRed) {
            invert = -1;
        }
    }

    @Override
    public void execute() {
        double dt = Timer.getTimestamp() - previousTimestamp;

        ChassisSpeeds driverSpeeds = new ChassisSpeeds(
            vX.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            vY.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            heading.getAsDouble() * drivetrainConfig.getMaxAngularVelocityRadiansPerSec()
        );

        Pose2d closestPose = new Pose2d(
            desiredAxis.getPointOnAxis(
                robotState.getEstimatedPose().getTranslation()
            ),
            desiredRotation
        );
        Logger.recordOutput("LockDriveAxis/closestPose", closestPose);

        Pose2d nextPoint = new Pose2d(
            desiredAxis.getPointOnAxis(
                new Translation2d(
                    robotState.getEstimatedPose().getTranslation().getX() + driverSpeeds.vxMetersPerSecond * dt,
                    robotState.getEstimatedPose().getTranslation().getY() + driverSpeeds.vyMetersPerSecond * dt
                )
            ),
            desiredRotation
        );
        Logger.recordOutput("LockDriveAxis/nextPoint", nextPoint);

        ChassisSpeeds goalSpeeds = new ChassisSpeeds(
            (nextPoint.getX() - closestPose.getX()) / dt,
            (nextPoint.getY() - closestPose.getY()) / dt,
            0
        );

        Logger.recordOutput("LockDriveAxis/goalSpeeds", goalSpeeds);

        super.targetPose = () -> nextPoint;
        super.endVelo = () -> goalSpeeds;
        
        super.execute();

        previousTimestamp = Timer.getTimestamp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}