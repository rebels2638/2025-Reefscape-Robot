package frc.robot.constants.vision;

public abstract class VisionConfigBase {
    public abstract String[] getNames(); 

    public abstract double getTrainslationDevBase();
    public abstract double getTranslationDevRotationExpoDenominator();
    public abstract double getTranslationDevTaScaler();

    public abstract double getObservationBufferSizeSeconds();

}
