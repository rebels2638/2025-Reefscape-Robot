package frc.robot.constants.robotState;

import edu.wpi.first.math.geometry.Translation2d;

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
    public Translation2d getCoralOffsetFromRobotCenter() {
        return new Translation2d(0.0,0); // -0.019
    }

    @Override
    public Translation2d getAlgayOffsetFromRobotCenter() {
        return new Translation2d(0, 0);
    }
}
