package frc.robot.constants.vision;

public abstract class VisionConfigBase {
    public abstract String[] getNames(); 

    public abstract double getTranslationDevBase();
    public abstract double getTranslationDevRotationExpoDenominator();
    public abstract double getTranslationDevTaScaler();

    public abstract double getObservationBufferSizeSeconds();

}
