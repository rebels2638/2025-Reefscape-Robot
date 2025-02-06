package frc.robot.constants.robotState;

import edu.wpi.first.math.geometry.Translation2d;

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
    public Translation2d getCoralOffsetFromRobotCenter() {
        return new Translation2d(0,0);
    }

    @Override
    public Translation2d getAlgayOffsetFromRobotCenter() {
        return new Translation2d(0, 0);
    }
}
