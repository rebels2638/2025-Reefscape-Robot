package frc.robot.commands.autoAlignment;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlignmentConstants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;

public class LockDriveAxis extends LinearDriveToPose {
    public static void main(String[] args) {
        Axis a = AlignmentConstants.kBARGE_AXIS;

        Translation2d b = a.getPointOnAxis(new Translation2d(5,6));
        System.out.println(b.getX() + " " + b.getY()); 

    }
    public static class Axis {
        public final Double x1, y1, x2, y2;
        public final Double m, b;
        public Axis(Double x1, Double y1, Double x2, Double y2) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;

            if (x2.equals(x1)) {
                this.m = Double.POSITIVE_INFINITY;
                this.b = x1;
            }
            else {
                this.m = (y2 - y1) / (x2 - x1);
                this.b = y1 - m * x1;
            }
        }

        public Axis(Double m, Double b) {
            this.x1 = Double.NaN;
            this.y1 = Double.NaN;
            this.x2 = Double.NaN;
            this.y2 = Double.NaN;

            this.m = m;
            this.b = b;
        }

        // will constrain between x1 and x2
        public Double getOutput(double input) {
            if (!Double.isNaN(x1) && !Double.isNaN(x2)) {
                input = RebelUtil.constrain(input, x1, x2);
            }
            if (m.isInfinite()) {
                return Double.POSITIVE_INFINITY;
            }
            return m * input + b;
        }

        public Axis getPerpendicularAxis(Double x3, Double y3) {
            Double m2;
            Double b2;
            if (this.m.isInfinite()) {
                m2 = 0.0;
                b2  = y3;
            }
            else {
                m2 = -1/this.m;
                b2 = y3 - m2 * x3;
            }

            return new Axis(
                m2,
                b2
            );
        }

        public Translation2d getIntersection(Axis other) {
            if (this.m.equals(other.m) || this.m.isInfinite() && other.m.isInfinite()) {
                return null;
            }

            Double x;
            Double y;
            if (this.m.isInfinite()) {
               x = this.b;
               y = other.getOutput(x);
            }
            else if (other.m.isInfinite()) {
                x = other.b;
                y = this.getOutput(x);
            }
            else {
                x = (this.b - other.b) / (other.m - this.m);
                y = getOutput(x);    
            }

            return new Translation2d(x, y);
        }

        public Translation2d getPointOnAxis(Translation2d p) {
            Double x3 = p.getX();
            Double y3 = p.getY();
            
            Axis perpendicularAxis = getPerpendicularAxis(x3, y3);
            Translation2d intersection = getIntersection(perpendicularAxis);
            if (intersection == null) {
                intersection = p;
            }

            if (!Double.isNaN(x1) && !Double.isNaN(x2) &&
                !Double.isNaN(y1) && !Double.isNaN(y2)) {
                return new Translation2d(
                    RebelUtil.constrain(intersection.getX(), x1, x2),
                    RebelUtil.constrain(intersection.getY(), y1, y2)
                );
            }
            else {
                return intersection;
            }
        }
    }

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private final SwerveDrivetrainConfigBase drivetrainConfig;

    private final DoubleSupplier vX, vY, heading; // Supplier functions for velocity inputs and heading.
    private int invert = 1;                       // Variable to invert direction based on alliance color.
    
    private double previousTimestamp = Timer.getTimestamp();

    private Supplier<Pose2d> targetPose;
    private Supplier<ChassisSpeeds> endVelo;
    
    public LockDriveAxis(XboxController xboxDriver) {
        super(() -> RobotState.getInstance().getEstimatedPose(), () -> new ChassisSpeeds());
        
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
            vX.getAsDouble() * drivetrainConfig.getMaxDrivetrainTranslationalVelocityMetersPerSec() * invert,
            vY.getAsDouble() * drivetrainConfig.getMaxDrivetrainTranslationalVelocityMetersPerSec() * invert,
            heading.getAsDouble() * drivetrainConfig.getMaxDrivetrainAngularVelocityRadiansPerSec()
        );

        Pose2d closestPose = new Pose2d(
            AlignmentConstants.kBARGE_AXIS.getPointOnAxis(
                robotState.getEstimatedPose().getTranslation()
            ),
            AlignmentConstants.kBARGE_ROTATION
        );
        Logger.recordOutput("LockDriveAxis/closestPose", closestPose);

        Pose2d nextPoint = new Pose2d(
            AlignmentConstants.kBARGE_AXIS.getPointOnAxis(
                new Translation2d(
                    robotState.getEstimatedPose().getTranslation().getX() + driverSpeeds.vxMetersPerSecond * dt,
                    robotState.getEstimatedPose().getTranslation().getY() + driverSpeeds.vyMetersPerSecond * dt
                )
            ),
            AlignmentConstants.kBARGE_ROTATION
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
