package frc.robot.constants.robotState;

public class RobotStatenConfigComp extends RobotStateConfigBase {
    private static RobotStatenConfigComp instance;
    public static RobotStatenConfigComp getInstance() {
        if (instance == null) {
            instance = new RobotStatenConfigComp();
        }

        return instance;
    }
    
    private RobotStatenConfigComp() {}

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

