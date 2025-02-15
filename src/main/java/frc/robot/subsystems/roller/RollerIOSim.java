package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.constants.roller.RollerConfigBase;
import frc.robot.lib.util.AlignmentUtil;

public class RollerIOSim implements RollerIO {
    private double appliedVolts = 0;

    private final RobotState robotState = RobotState.getInstance();
    
    private double rollerRunStartTime = 0;

    public RollerIOSim() {}

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.rollerVelocityRadPerSec = appliedVolts * 20;
        inputs.rollerAppliedVolts = appliedVolts;
        
        if (Math.abs(inputs.rollerVelocityRadPerSec) == 0) {
            rollerRunStartTime = Timer.getTimestamp();
        }
        double timeSinceRollerRun = Timer.getTimestamp() - rollerRunStartTime;
        Logger.recordOutput("Roller/timeSinceRollerRun", timeSinceRollerRun);

        inputs.inRoller = 
            !inputs.inRoller ?
                AlignmentUtil.getClosestSourcePose().getTranslation().getDistance(robotState.getEstimatedPose().getTranslation()) < 0.40 &&
                timeSinceRollerRun >= 2 : 
                timeSinceRollerRun < 0.5;

    }

    @Override
    public void setTorqueCurrentFOC(double current) {
        appliedVolts = current;
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVolts = voltage;
    }
}
