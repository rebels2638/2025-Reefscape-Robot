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
    public double getTranslationDevDenominator() {
        return 60;
    }

    @Override
    public double getObservationBufferSizeSeconds() {
        return 1;
    }
}
