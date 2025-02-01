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
    public double getVisionTrainslationDevBase() {
        return 0;
    }

    @Override
    public double getOdomTrainslationDevBase() {
        return 0;
    }
}
