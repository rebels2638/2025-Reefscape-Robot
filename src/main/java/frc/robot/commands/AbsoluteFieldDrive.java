package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds; // Class to handle chassis speed calculations.
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation; // Driver station for accessing driver settings like alliance.
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command; // Base class for commands.
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.SwerveConfigBase;
import frc.robot.constants.swerve.SwerveCompConfig;
import frc.robot.constants.swerve.SwerveSimConfig;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive; // Swerve drive subsystem for robot movement.

public class AbsoluteFieldDrive extends Command {

    private static final double deadband = 0.2;
    private static final double rotation_coeff = 0.75;
    private final SwerveDrive swerveSubsystem;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier angularSupplier;
    private final BooleanSupplier joystick;
    private final SwerveConfigBase config;

    public AbsoluteFieldDrive(
        SwerveDrive swerve,
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier ang,
        BooleanSupplier joystick,
        SwerveConfigBase config) {
            this.config = config;
            this.swerveSubsystem = swerve;
            this.xSupplier = x;
            this.ySupplier = y;
            this.joystick = joystick;
            this.angularSupplier = ang;

        addRequirements(swerve); // Specify that this command requires the swerve subsystem.
    }

    @Override
    public void execute() {
        double xMag = MathUtil.applyDeadband(xSupplier.getAsDouble(), deadband);
        double yMag = MathUtil.applyDeadband(ySupplier.getAsDouble(), deadband);
        double angMag = MathUtil.applyDeadband(angularSupplier.getAsDouble(), deadband);

        xMag = Math.copySign(xMag*xMag, xMag);
        yMag = Math.copySign(yMag*yMag, yMag);
        angMag = Math.copySign(angMag*angMag, angMag);

        double Vx = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 
            -xMag * config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC : 
            xMag * config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC;

        double Vy = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ?
            -yMag * config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC :
            yMag * config.getSwerveDrivetrainConfig().kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC;

        double omega = angMag * 12; // drivetrain_omega_max
        omega *= joystick.getAsBoolean() ? 1.0 : rotation_coeff;

        ChassisSpeeds setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(Vx, Vy, omega, RobotState.getInstance().getOdometryPose().getRotation());
        swerveSubsystem.setTargetSpeed(setpoint);

    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // This command runs indefinitely until interrupted.
    }
}
