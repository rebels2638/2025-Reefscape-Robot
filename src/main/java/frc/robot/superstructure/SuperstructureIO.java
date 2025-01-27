package frc.robot.superstructure;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.constants.Constants.SuperstructureConfig;
import frc.robot.superstructure.Superstructure.AlgayState;
import frc.robot.superstructure.Superstructure.CoralState;

public interface SuperstructureIO {
    @AutoLog
    class SuperstructureIOInputs {
        public CoralState coralState = SuperstructureConfig.kCORAL_STARTING_STATE;
        public AlgayState algayState = SuperstructureConfig.kALGAY_STARTING_STATE;
    }

    public default void updateInputs(SuperstructureIOInputs inputs) {}
}
