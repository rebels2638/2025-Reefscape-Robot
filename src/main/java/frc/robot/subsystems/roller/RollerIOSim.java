package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.simulation.UltrasonicSim;
import frc.robot.RobotState;
import frc.robot.lib.util.AlignmentUtil;

public class RollerIOSim implements RollerIO {
    private double appliedVolts = 0;

    private final RobotState robotState = RobotState.getInstance();
    
    private double rollerRunStartTime = 0;

    private final UltrasonicSim canRangeSim;
    private final Ultrasonic canRangeSensor;

    public RollerIOSim() {
        //sensor sim is for the yn streets onfg 
        canRangeSensor = new Ultrasonic(1, 2);
        canRangeSensor.setEnabled(true);

        canRangeSim = new UltrasonicSim(canRangeSensor);
        canRangeSim.setRangeInches(6.5);
        canRangeSim.setRangeValid(true);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.rollerVelocityRadPerSec = appliedVolts * 20;
        inputs.rollerAppliedVolts = appliedVolts;
        
        if (Math.abs(inputs.rollerVelocityRadPerSec) == 0) {
            rollerRunStartTime = Timer.getTimestamp();
        }
        double timeSinceRollerRun = Timer.getTimestamp() - rollerRunStartTime;
        Logger.recordOutput("Roller/timeSinceRollerRun", timeSinceRollerRun);

        inputs.inRoller = canRangeSensor.isRangeValid();

        inputs.inRoller = 
            !inputs.inRoller ?
                AlignmentUtil.getClosestSourcePose().getTranslation().getDistance(robotState.getEstimatedPose().getTranslation()) < 0.20 &&
                timeSinceRollerRun >= 0.2 : 
                timeSinceRollerRun < 0.2;
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
