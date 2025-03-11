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
        return 0;
    }

    @Override
    public double getOdomTranslationDevBase() {
        return 0;
    }

    @Override
    public double getMaxElevatorExtensionVelocityMeterPerSec() {
        return 4.5;
    }
    
    @Override
    public double getMaxElevatorExtensionAccelerationMetersPerSecPerSec() {
        return 7.2;
    }

    @Override
    public double getMaxRotationalVelocityRadPerSecPerSec() {
        return 2 * Math.PI * 0.5;
    }

    @Override
    public int getMinLocalVisionObservationCount() {
        return -1;
    }
}
