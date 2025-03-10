package frc.robot.constants.vision;

public class VisionConfigProto extends VisionConfigBase {
    private static VisionConfigProto instance;
    public static VisionConfigProto getInstance() {
        if (instance == null) {
            instance = new VisionConfigProto();
        }

        return instance;
    }
    
    private VisionConfigProto() {}

    @Override
    public String[] getNames() {
        return new String[] {
            "limelight-back"
        };
    }

    @Override
    public double getTranslationDevBase() {
        return 0.05;
    }

    @Override
    public double getTranslationDevRotationExpoDenominator() {
        return 16;
    }
    
    @Override
    public double getTranslationDevRotationExpo() {
        return 6;
    }

    @Override
    public double getTranslationDevTaScaler() {
        return 15;
    }

    @Override
    public double getObservationBufferSizeSeconds() {
        return 1;
    }
}
