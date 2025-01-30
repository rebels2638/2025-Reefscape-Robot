package frc.robot.constants.swerve.pathplannerConfigs;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.constants.swerve.moduleConfigs.sim.SwerveModuleGeneralConfigSim;

public class SwervePathplannerConfigSim extends SwervePathplannerConfigBase{

    public static SwervePathplannerConfigSim instance = null;
    public static SwervePathplannerConfigSim getInstance() {
        if (instance == null) {
            instance = new SwervePathplannerConfigSim();
        }
        return instance;
    }

    private final RobotConfig robotConfig;

    private SwervePathplannerConfigSim() {
        this.robotConfig = new RobotConfig(
            27.88,
            3.5,
            new ModuleConfig(
                SwerveModuleGeneralConfigSim.getInstance().getDriveWheelRadiusMeters(), 
                5.4, 
                1.2, 
                DCMotor.getFalcon500(1).
                    withReduction(
                        SwerveModuleGeneralConfigSim.getInstance().getDriveMotorToOutputShaftRatio()
                    ),
                SwerveModuleGeneralConfigSim.getInstance().getDriveStatorCurrentLimit(), 
                1
            ),
            SwerveDrivetrainConfigSim.getInstance().getFrontLeftPositionMeters(), 
            SwerveDrivetrainConfigSim.getInstance().getFrontRightPositionMeters(), 
            SwerveDrivetrainConfigSim.getInstance().getBackLeftPositionMeters(), 
            SwerveDrivetrainConfigSim.getInstance().getBackRightPositionMeters()
        );
    }

    @Override
    public RobotConfig getRobotConfig() {
        return this.robotConfig;
    }
}
