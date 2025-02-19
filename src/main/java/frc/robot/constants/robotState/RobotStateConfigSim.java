package frc.robot.constants.robotState;

public class RobotStateConfigSim extends RobotStateConfigBase {
    private static RobotStateConfigSim instance;
    public static RobotStateConfigSim getInstance() {
        if (instance == null) {
            instance = new RobotStateConfigSim();
        }

        return instance;
    }
    
    private RobotStateConfigSim() {}

    @Override
    public double getVisionTranslationDevBase() {
        return 0.15;
    }

    @Override
    public double getOdomTranslationDevBase() {
        return 0.05;
    }

    @Override
    public double getMaxElevatorExtensionVelocity() {
        return 1;
    }
    
    @Override
    public double getMaxElevatorExtensionAcceleration() {
        return 1;
    }

    @Override
    public double getMinElevatorExtensionAcceleration() {
        return 1;
    }
}
