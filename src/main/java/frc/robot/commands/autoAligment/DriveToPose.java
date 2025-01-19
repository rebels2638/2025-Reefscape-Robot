package frc.robot.commands.autoAligment;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.Constants;
import frc.robot.constants.ElementConstants;
import frc.robot.constants.swerve.SwerveModuleConfig.GeneralConfig;
import frc.robot.constants.swerve.SwerveCompConfig;
import frc.robot.constants.swerve.SwerveSimConfig;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class DriveToPose extends Command {
    private SwerveDrive swerveDrive;

    private Pose2d targetPose = ElementConstants.kPOSE_ARR[0];
    private Pose2d currentPose;

    private GeneralConfig generalConfig = new SwerveSimConfig().getSharedGeneralConfig();

    private static final double MAX_ANGULAR_ACCEL_RPS_SQUARED = 3.5;

    double prevTimeInputs;
    
    private ProfiledPIDController translationalPIDX = new ProfiledPIDController(generalConfig.kDRIVE_KP, 
                                                                                generalConfig.kDRIVE_KI,
                                                                                generalConfig.kDRIVE_KD, 
                                                    new TrapezoidProfile.Constraints(
                                                                                generalConfig.kDRIVE_MAX_VELOCITY_METERS_PER_SEC,
                                                                                generalConfig.kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC));
    private ProfiledPIDController translationalPIDY = new ProfiledPIDController(generalConfig.kDRIVE_KP, 
                                                                                generalConfig.kDRIVE_KI,
                                                                                generalConfig.kDRIVE_KD, 
                                                    new TrapezoidProfile.Constraints(
                                                                                generalConfig.kDRIVE_MAX_VELOCITY_METERS_PER_SEC,
                                                                                generalConfig.kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC));
    private ProfiledPIDController rotationalPID = new ProfiledPIDController(generalConfig.kSTEER_KP,
                                                                                generalConfig.kSTEER_KI, 
                                                                                generalConfig.kDRIVE_KP, 
                                                    new TrapezoidProfile.Constraints(
                                                                                generalConfig.kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_ROTATIONS_PER_SEC, 
                                                                                MAX_ANGULAR_ACCEL_RPS_SQUARED));

    public DriveToPose(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
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
        Logger.recordOutput("DriveToPose/targetPose", targetPose);

        translationalPIDX.reset(currentPose.getTranslation().getX(), swerveDrive.getMeasuredFieldRelativeSpeeds().vxMetersPerSecond);
        translationalPIDY.reset(currentPose.getTranslation().getY(), swerveDrive.getMeasuredFieldRelativeSpeeds().vyMetersPerSecond);
        rotationalPID.reset(currentPose.getRotation().getRadians(), swerveDrive.getMeasuredFieldRelativeSpeeds().omegaRadiansPerSecond);

        translationalPIDX.setGoal(targetPose.getTranslation().getX());
        translationalPIDY.setGoal(targetPose.getTranslation().getY());
        rotationalPID.setGoal(targetPose.getRotation().getRadians());

        translationalPIDX.setTolerance(0.05);
        translationalPIDY.setTolerance(0.05);
        rotationalPID.setTolerance(Math.toRadians(1));

        rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        double dt = Timer.getFPGATimestamp() - prevTimeInputs;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        currentPose = swerveDrive.getPose();

        double feedbackX = translationalPIDX.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
        double feedbackY = translationalPIDY.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());
        double feedbackTheta = rotationalPID.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        double feedforwardX = translationalPIDX.getSetpoint().velocity;
        double feedforwardY = translationalPIDY.getSetpoint().velocity;
        double feedforwardTheta = rotationalPID.getSetpoint().velocity;

        chassisSpeeds.vxMetersPerSecond = feedforwardX + feedbackX;  
        chassisSpeeds.vyMetersPerSecond = feedbackY + feedforwardY;
        chassisSpeeds.omegaRadiansPerSecond = feedbackTheta + feedforwardTheta;

        Logger.recordOutput("DriveToPose/calculatedAngle", calculateAngle(currentPose, targetPose));
        Logger.recordOutput("DriveToPose/calculatedDistance", calculateDistance(currentPose, targetPose));

        //GRAH WEFSASF GDSF GSRJKAFSKHSFAHFSAGHLASDLH VX AND VY WORK PLEASE BR OWAEEOGUFFIFLHJB
        //it works now lol :3
        swerveDrive.driveFieldRelative(chassisSpeeds);

        prevTimeInputs = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            swerveDrive.driveFieldRelative(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        double positionTolerance = .1;
        if (calculateDistance(swerveDrive.getPose(), targetPose) < positionTolerance && 
            Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()) < Math.toRadians(1.3)) {
            return true;
        }
        return false;
    }    

    private static double calculateDistance(Pose2d currentPose, Pose2d targetPose) {
        return currentPose.getTranslation().getDistance(targetPose.getTranslation());
    }
    //experiment with .relativeTo for poses, this doesnt work how i want it 2 rn
    private static double calculateAngle(Pose2d currentPose, Pose2d targetPose) {
        // return Math.toDegrees(Math.atan2(targetPose.getTranslation().getY() - currentPose.getTranslation().getY(), 
        //                                  targetPose.getTranslation().getX() - currentPose.getTranslation().getX()));
        return Math.toDegrees(Math.atan2(currentPose.relativeTo(targetPose).getY(), currentPose.relativeTo(targetPose).getX()));
    }
}
