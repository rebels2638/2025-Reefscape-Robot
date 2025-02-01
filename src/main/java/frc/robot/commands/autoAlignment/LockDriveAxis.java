package frc.robot.commands.autoAlignment;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.lib.util.RebelUtil;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlignmentConstants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;

public class LockDriveAxis extends Command {
    public static class Axis {
        public final Double x1, y1, x2, y2;
        public final Double m, b;
        public Axis(Double x1, Double y1, Double x2, Double y2) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;

            if (x2 == x1) {
                this.m = Double.POSITIVE_INFINITY;
                this.b = 0.0;
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
            if (this.m == other.m) {
                return null;
            }

            Double x = (this.b - other.b) / (this.m - other.m);
            Double y = getOutput(x);

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

    private final SwerveDrive swerve = SwerveDrive.getInstance();             // Reference to the swerve drive subsystem.
    private final DoubleSupplier vX, vY, heading; // Supplier functions for velocity inputs and heading.
    private int invert = 1;                       // Variable to invert direction based on alliance color.
    
    private double previousTimestamp = Timer.getTimestamp();

    public LockDriveAxis(XboxController xboxDriver) {
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

        addRequirements(swerveDrive); // Specify that this command requires the swerve subsystem.
    }

    @Override
    public void initialize() {
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
        double dt = previousTimestamp - Timer.getTimestamp();

        Pose2d closestPose = new Pose2d(
            AlignmentConstants.kBARGE_AXIS.getPointOnAxis(
                robotState.getEstimatedPose().getTranslation()
            ),
            AlignmentConstants.kBARGE_ROTATION
        );
        
        ChassisSpeeds driverSpeeds = new ChassisSpeeds(
            vX.getAsDouble() * drivetrainConfig.getMaxDrivetrainTranslationalVelocityMetersPerSec(),
            vY.getAsDouble() * drivetrainConfig.getMaxDrivetrainTranslationalVelocityMetersPerSec(),
            heading.getAsDouble() * drivetrainConfig.getMaxDrivetrainAngularVelocityRadiansPerSec()
        );

        Pose2d driverTranslatedPose = new Pose2d(
            new Translation2d(
                closestPose.getX() + driverSpeeds.vxMetersPerSecond * dt,
                closestPose.getY() + driverSpeeds.vyMetersPerSecond * dt
            ),
            AlignmentConstants.kBARGE_ROTATION
        );


        previousTimestamp = Timer.getTimestamp();
    }
}
