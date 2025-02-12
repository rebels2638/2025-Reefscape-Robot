package frc.robot.constants.robotState;

import java.util.concurrent.TransferQueue;

import edu.wpi.first.math.geometry.Translation2d;

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
}
