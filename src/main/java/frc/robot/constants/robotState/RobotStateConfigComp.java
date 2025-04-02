package frc.robot.constants.robotState;


public class RobotStateConfigComp extends RobotStateConfigBase {
    private static RobotStateConfigComp instance;
    public static RobotStateConfigComp getInstance() {
        if (instance == null) {
            instance = new RobotStateConfigComp();
        }

        return instance;
    }
    
    private RobotStateConfigComp() {}

    @Override
    public double getVisionTranslationDevBase() {
        return 0.02;
    }

    @Override
    public double getOdomTranslationDevBase() {
        return 0.005;
    }

    @Override
    public double getMaxElevatorExtensionVelocityMeterPerSec() {
        return 1;
    }
    
    @Override
    public double getMaxElevatorExtensionAccelerationMetersPerSecPerSec() {
        return 1;
    }

    @Override
    public double getMaxRotationalVelocityRadPerSecPerSec() {
        return 1;
    }

    @Override
    public int getMinLocalVisionObservationCount() {
        return 70;
    }
}
