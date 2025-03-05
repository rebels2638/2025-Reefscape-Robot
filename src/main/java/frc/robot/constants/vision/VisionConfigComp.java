package frc.robot.constants.vision;

public class VisionConfigComp extends VisionConfigBase {
    private static VisionConfigComp instance;
    public static VisionConfigComp getInstance() {
        if (instance == null) {
            instance = new VisionConfigComp();
        }

        return instance;
    }
    
    private VisionConfigComp() {}

    @Override
    public String[] getNames() {
        return new String[] {
            "limelight-right",
            "limelight-left"
        };
    }

    @Override
    public double getTranslationDevBase() {
        return 0.05; // 0.05
    }

    @Override
    public double getTranslationDevRotationExpoDenominator() {
        return 16;
    }

    @Override
    public double getTranslationDevTaScaler() {
        return 0.0002;
    }

    @Override
    public double getObservationBufferSizeSeconds() {
        return 2;
    }
}
