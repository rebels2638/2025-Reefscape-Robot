package frc.robot.constants.swerve.pathplannerConfigs;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleGeneralConfigProto;

public class SwervePathplannerConfigProto extends SwervePathplannerConfigBase{

    public static SwervePathplannerConfigProto instance = null;
    public static SwervePathplannerConfigProto getInstance() {
        if (instance == null) {
            instance = new SwervePathplannerConfigProto();
        }
        return instance;
    }

    private final RobotConfig robotConfig;

    private SwervePathplannerConfigProto() {
        this.robotConfig = new RobotConfig(
            27.88,
            3.5,
            new ModuleConfig(
                SwerveModuleGeneralConfigProto.getInstance().getDriveWheelRadiusMeters(), 
                5.4, 
                1.2, 
                DCMotor.getFalcon500(1).
                    withReduction(
                        SwerveModuleGeneralConfigProto.getInstance().getDriveMotorToOutputShaftRatio()
                    ),
                SwerveModuleGeneralConfigProto.getInstance().getDriveStatorCurrentLimit(), 
                1
            ),
            SwerveDrivetrainConfigProto.getInstance().getFrontLeftPositionMeters(), 
            SwerveDrivetrainConfigProto.getInstance().getFrontRightPositionMeters(), 
            SwerveDrivetrainConfigProto.getInstance().getBackLeftPositionMeters(), 
            SwerveDrivetrainConfigProto.getInstance().getBackRightPositionMeters()
        );
    }

    @Override
    public RobotConfig getRobotConfig() {
        return this.robotConfig;
    }
}
