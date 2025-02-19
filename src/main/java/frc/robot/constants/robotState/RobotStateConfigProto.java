package frc.robot.constants.robotState;

public class RobotStateConfigProto extends RobotStateConfigBase {
    private static RobotStateConfigProto instance;
    public static RobotStateConfigProto getInstance() {
        if (instance == null) {
            instance = new RobotStateConfigProto();
        }

        return instance;
    }
    
    private RobotStateConfigProto() {}

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
