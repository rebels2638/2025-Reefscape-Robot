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
            "limelight"
        };
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
