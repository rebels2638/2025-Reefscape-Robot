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
    public double getVisionTrainslationDevBase() {
        return 0;
    }

    @Override
    public double getOdomTrainslationDevBase() {
        return 0;
    }
}
