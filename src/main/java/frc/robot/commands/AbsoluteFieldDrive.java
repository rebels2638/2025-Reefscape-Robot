package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds; // Class to handle chassis speed calculations.
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command; // Base class for commands.
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive; // Swerve drive subsystem for robot movement.

public class AbsoluteFieldDrive extends Command {

    private final SwerveDrive swerve = SwerveDrive.getInstance();             // Reference to the swerve drive subsystem.
    private final DoubleSupplier vX, vY, heading; // Supplier functions for velocity inputs and heading.
    private int invert = 1;                        // Variable to invert direction based on alliance color.
    
    private final SwerveDrivetrainConfigBase drivetrainConfig;

    private double lastTheta = 0;
    private double lastTime = 0;

    // Constructor to initialize the AbsoluteFieldDrive command.
    public AbsoluteFieldDrive(XboxController xboxDriver) {
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

        addRequirements(swerve); // Specify that this command requires the swerve subsystem.
    }

    // Called when the command is initialized.
    @Override
    public void initialize() {
        invert = Constants.shouldFlipPath() ? -1 : 1;
        lastTime = Timer.getTimestamp();
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        double dt = Timer.getTimestamp() - lastTime;

        // Calculate speeds based on input and max speed constants.
        ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds(
            vX.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            vY.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            heading.getAsDouble() * drivetrainConfig.getMaxAngularVelocityRadiansPerSec()
        );
        Logger.recordOutput("AbsoluteFieldDrive/desiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);
        
        double mag = Math.hypot(desiredFieldRelativeSpeeds.vxMetersPerSecond, desiredFieldRelativeSpeeds.vyMetersPerSecond);
        double theta =  Math.atan2(desiredFieldRelativeSpeeds.vyMetersPerSecond, desiredFieldRelativeSpeeds.vxMetersPerSecond);
        double limThetaSec = RebelUtil.constrain(Math.toRadians(360 - mag * 120), 0.01, 3000);
        double delta = RebelUtil.constrain(theta - lastTheta, -limThetaSec * dt, limThetaSec * dt);
        double limTheta = lastTheta + delta;

        ChassisSpeeds limSpeeds = new ChassisSpeeds();
        limSpeeds.vxMetersPerSecond = Math.cos(limTheta) * mag;
        limSpeeds.vyMetersPerSecond = Math.sin(limTheta) * mag;
        limSpeeds.omegaRadiansPerSecond = desiredFieldRelativeSpeeds.omegaRadiansPerSecond;
        Logger.recordOutput("AbsoluteFieldDrive/limFieldRelativeSpeeds", limSpeeds);

        swerve.driveFieldRelative(desiredFieldRelativeSpeeds); // Drive the robot using the calculated speeds.

        lastTheta = theta;
        lastTime = Timer.getTimestamp();     

    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Cleanup or reset logic can be added here if necessary.
        // swerve.disableRotationLock();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // This command runs indefinitely until interrupted.
    }
}
