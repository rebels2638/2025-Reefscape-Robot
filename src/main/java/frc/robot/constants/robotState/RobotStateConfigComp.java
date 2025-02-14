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
        return 0.15;
    }

    @Override
    public double getOdomTranslationDevBase() {
        return 0.05;
    }
}
