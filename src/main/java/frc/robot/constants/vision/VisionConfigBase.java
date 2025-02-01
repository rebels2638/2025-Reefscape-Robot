package frc.robot.constants.vision;

public abstract class VisionConfigBase {
    public abstract String[] getNames(); 
    public abstract double getTranslationDevDenominator();
    public abstract double getObservationBufferSizeSeconds();

}
