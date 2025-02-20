package frc.robot.commands.autoAlignment;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.lib.input.XboxController;

public class OverridePathplannerFeedbackXboxController extends OverridePathplannerFeedback {
    private static final SwerveDrivetrainConfigBase drivetrainConfig;
    static {
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
    }

    public OverridePathplannerFeedbackXboxController(XboxController controller) {
        super(
            () -> new ChassisSpeeds(
                -MathUtil.applyDeadband(controller.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND) * 
                drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * 
                (Constants.shouldFlipPath() ? -1 : 1),
                -MathUtil.applyDeadband(controller.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND) * 
                drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * 
                (Constants.shouldFlipPath() ? -1 : 1),
                -MathUtil.applyDeadband(controller.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND) * 
                drivetrainConfig.getMaxAngularVelocityRadiansPerSec()
            )
        );
    }
}