package frc.robot.constants.swerve.pathplannerConfigs;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleGeneralConfigComp;

public class SwervePathplannerConfigComp extends SwervePathplannerConfigBase{

    public static SwervePathplannerConfigComp instance = null;
    public static SwervePathplannerConfigComp getInstance() {
        if (instance == null) {
            instance = new SwervePathplannerConfigComp();
        }
        return instance;
    }

    private final RobotConfig robotConfig;

    public SwervePathplannerConfigComp() {
        this.robotConfig = new RobotConfig(
            27.88,
            3.5,
            new ModuleConfig(
                SwerveModuleGeneralConfigComp.getInstance().getDriveWheelRadiusMeters(), 
                5.4, 
                1.2, 
                DCMotor.getFalcon500(1).
                    withReduction(
                        SwerveModuleGeneralConfigComp.getInstance().getDriveMotorToOutputShaftRatio()
                    ),
                SwerveModuleGeneralConfigComp.getInstance().getDriveStatorCurrentLimit(), 
                1
            ),
            SwerveDrivetrainConfigComp.getInstance().getFrontLeftPositionMeters(), 
            SwerveDrivetrainConfigComp.getInstance().getFrontRightPositionMeters(), 
            SwerveDrivetrainConfigComp.getInstance().getBackLeftPositionMeters(), 
            SwerveDrivetrainConfigComp.getInstance().getBackRightPositionMeters()
        );
    }

    @Override
    public RobotConfig getRobotConfig() {
        return this.robotConfig;
    }
}
