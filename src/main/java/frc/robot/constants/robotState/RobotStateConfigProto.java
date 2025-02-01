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
    public double getVisionTrainslationDevBase() {
        return 0.15;
    }

    @Override
    public double getOdomTrainslationDevBase() {
        return 0.05;
    }
}
