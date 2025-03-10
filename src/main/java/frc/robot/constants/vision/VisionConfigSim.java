package frc.robot.constants.vision;

public class VisionConfigSim extends VisionConfigBase {
    private static VisionConfigSim instance;
    public static VisionConfigSim getInstance() {
        if (instance == null) {
            instance = new VisionConfigSim();
        }

        return instance;
    }
    
    private VisionConfigSim() {}

    @Override
    public String[] getNames() {
        return new String[] {};
    }

    @Override
    public double getTranslationDevBase() {
        return 0.5;
    }
    
    @Override
    public double getTranslationDevRotationExpoDenominator() {
        return 60;
    }

    @Override
    public double getTranslationDevRotationExpo() {
        return 6;
    }

    @Override
    public double getTranslationDevTaScaler() {
        return 0;
    }

    @Override
    public double getObservationBufferSizeSeconds() {
        return 1;
    }
}
