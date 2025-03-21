package frc.robot.constants.swerve.drivetrainConfigs;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.constants.swerve.moduleConfigs.sim.SwerveModuleGeneralConfigSim;
import frc.robot.lib.util.LoggedTunableNumber;

public class SwerveDrivetrainConfigComp extends SwerveDrivetrainConfigBase {

    public static SwerveDrivetrainConfigComp instance = null;

    private static LoggedTunableNumber[] AutoAlignTranslationalPID = {
        new LoggedTunableNumber(
            "AutoAlign/translational/kP", 2.5
        ),
        new LoggedTunableNumber(
            "AutoAlign/translational/kI", 0.0
        ),
        new LoggedTunableNumber(
            "AutoAlign/translational/kD", 0.2
        ),
    };
    private static LoggedTunableNumber[] AutoAlignRotationalPID = {
        new LoggedTunableNumber(
            "AutoAlign/rotational/kP", 2.5
        ),
        new LoggedTunableNumber(
            "AutoAlign/rotational/kI", 0.0
        ),
        new LoggedTunableNumber(
            "AutoAlign/rotational/kD", 0.0
        ),
    };

    public static SwerveDrivetrainConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrainConfigComp();
        }

        return instance;
    }

    private SwerveDrivetrainConfigComp() {}
    
    @Override
    public double getMaxTranslationalVelocityMetersPerSec() {
        return 4.5;
    }

    @Override
    public double getMaxTranslationalAccelerationMetersPerSecSec() {
        return 5.4;
    }

    @Override
    public double getMaxAngularVelocityRadiansPerSec() {
        return 6.28; // 3.7
    }

    @Override
    public double getMaxAngularAccelerationRadiansPerSecSec() {
        return 12.0;
    }

    @Override
    public double getMaxModuleVelocity() {
        return 4.5;
    }

    @Override
    public Translation2d getFrontLeftPositionMeters() {
        return new Translation2d(0.23, 0.23);
    }

    @Override
    public Translation2d getFrontRightPositionMeters() {
        return new Translation2d(0.23, -0.23);
    }

    @Override
    public Translation2d getBackLeftPositionMeters() {
        return new Translation2d(-0.23, 0.23);
    }

    @Override
    public Translation2d getBackRightPositionMeters() {
        return new Translation2d(-0.23, -0.23);
    }

    @Override
    public RobotConfig getRobotConfig() {
        return new RobotConfig(
            52.16,
            5,
            new ModuleConfig(
                SwerveModuleGeneralConfigSim.getInstance().getDriveWheelRadiusMeters(), 
                4.2, 
                1.2, 
                DCMotor.getKrakenX60(1).
                    withReduction(
                        SwerveModuleGeneralConfigSim.getInstance().getDriveMotorToOutputShaftRatio()
                    ),
                SwerveModuleGeneralConfigSim.getInstance().getDriveStatorCurrentLimit(), 
                1
            ),
            getFrontLeftPositionMeters(), 
            getFrontRightPositionMeters(), 
            getBackLeftPositionMeters(), 
            getBackRightPositionMeters()
        );
    }

    @Override
    public PIDConstants getPathplannerDrivePIDConfig() {
        return new PIDConstants(3,0,0.1 ,0);
    }

    @Override
    public PIDConstants getPathplannerSteerPIDConfig() {
        return new PIDConstants(4,0,0.03,0);
    }

    @Override
    public double getRotationCompensationCoefficient() {
        return 0.0;
    }

    @Override
    public PIDController getAutoAlignProfiledTranslationController() {
        PIDController p = new PIDController(2.5, 0.2, 0);
        if (AutoAlignTranslationalPID[0].hasChanged(hashCode())) {
            p.setP(AutoAlignTranslationalPID[0].getAsDouble());
        }
        if (AutoAlignTranslationalPID[0].hasChanged(hashCode())) {
            p.setI(AutoAlignTranslationalPID[1].getAsDouble());
        }
        if (AutoAlignTranslationalPID[0].hasChanged(hashCode())) {
            p.setD(AutoAlignTranslationalPID[2].getAsDouble());
        }
        p.setTolerance(getAutoAlignTranslationTolerance(), getAutoAlignTranslationVeloTolerance());

        return p;
    }

    @Override
    public PIDController getAutoAlignProfiledRotationController() {
        PIDController p = new PIDController(2.5, 0, 0);
        if (AutoAlignRotationalPID[0].hasChanged(hashCode())) {
            p.setP(AutoAlignRotationalPID[0].getAsDouble());
        }
        if (AutoAlignTranslationalPID[0].hasChanged(hashCode())) {
            p.setI(AutoAlignRotationalPID[1].getAsDouble());
        }
        if (AutoAlignTranslationalPID[0].hasChanged(hashCode())) {
            p.setD(AutoAlignRotationalPID[2].getAsDouble());
        }
        p.setTolerance(getAutoAlignRotationTolerance(), getAutoAlignRotationVeloTolerance());
        p.enableContinuousInput(-Math.PI, Math.PI);

        return p;
    }

    @Override
    public double getAutoAlignTranslationTolerance() {
        return 0.025;
    }

    @Override
    public double getAutoAlignTranslationVeloTolerance() {
        return 0.07;
    }

    @Override
    public double getAutoAlignRotationTolerance() {
        return Math.toRadians(3);
    }

    @Override
    public double getAutoAlignRotationVeloTolerance() {
        return Math.toRadians(3);
    }
    
    @Override
    public double getBumperLengthMeters() {
        return 0.75;
    }

    @Override
    public Translation2d getBranchOffsetFromRobotCenter() {
        return new Translation2d(0,-0.015); // increasing the y value will move the robot to the left of the branch
    }

    @Override
    public Translation2d getAlgayOffsetFromRobotCenter() {
        return new Translation2d(0.0, 0.02);
    }

    @Override
    public double getMaxAlignmentTranslationVeloMetersPerSec() {
        return 2;
    }

    @Override
    public double getMaxAlignmentRotationVeloRadPerSec() {
        return 3.7;
    }

    @Override
    public double getMaxAlignmentTranslationalAcelMetersPerSecPerSec() {
        return 3;
    }

    @Override
    public double getMaxAlignmentRotationAcelRadPerSecPerSec() {
        return 10;
    }

    @Override
    public double getAlgayRecessPoseOffset() {
        return 0.5;
    }

}