package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds; // Class to handle chassis speed calculations.
import edu.wpi.first.wpilibj.DriverStation; // Driver station for accessing driver settings like alliance.
import edu.wpi.first.wpilibj2.command.Command; // Base class for commands.
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive; // Swerve drive subsystem for robot movement.

public class AbsoluteFieldDrive extends Command {

    private final SwerveDrive swerve = SwerveDrive.getInstance();             // Reference to the swerve drive subsystem.
    private final DoubleSupplier vX, vY, heading; // Supplier functions for velocity inputs and heading.
    private int invert = 1;                        // Variable to invert direction based on alliance color.
    
    private final SwerveDrivetrainConfigBase drivetrainConfig;

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
        boolean isRed = false;
        var alliance = DriverStation.getAlliance(); // Get the current alliance.

        // Check if the alliance is Red and set invert accordingly.
        if (alliance.isPresent()) {
            isRed = alliance.get() == DriverStation.Alliance.Red;
        }

        // Invert drive direction if the robot is on the Red alliance.
        if (isRed) {
            invert = -1;
        }
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        // Calculate speeds based on input and max speed constants.
        ChassisSpeeds speeds = new ChassisSpeeds(
            vX.getAsDouble() * drivetrainConfig.getMaxDrivetrainTranslationalVelocityMetersPerSec() * invert,
            vY.getAsDouble() * drivetrainConfig.getMaxDrivetrainTranslationalVelocityMetersPerSec() * invert,
            heading.getAsDouble() * drivetrainConfig.getMaxDrivetrainAngularVelocityRadiansPerSec()
        );

        swerve.driveFieldRelative(speeds); // Drive the robot using the calculated speeds.
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Cleanup or reset logic can be added here if necessary.
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // This command runs indefinitely until interrupted.
    }
}
