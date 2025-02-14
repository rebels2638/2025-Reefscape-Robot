package frc.robot.subsystems.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.roller.RollerConfigBase;

public class RollerIOSim implements RollerIO {
    private DCMotor rollerGearBox = DCMotor.getKrakenX60Foc(1);

    private FlywheelSim rollerSim;

    private double prevTimeInputs = 0;
    private double appliedVolts = 0;
    
    public RollerIOSim(RollerConfigBase config) {
        rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                rollerGearBox,
                0.001,
                5
            ),
            rollerGearBox
        );
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        double dt = Timer.getTimestamp() - prevTimeInputs;
        rollerSim.update(dt);

        inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
        inputs.rollerAppliedVolts = appliedVolts;

        prevTimeInputs = Timer.getTimestamp();
    }

    @Override
    public void setTorqueCurrentFOC(double current) {
        appliedVolts = current;
        rollerSim.setInputVoltage(current);
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVolts = voltage;
        rollerSim.setInputVoltage(voltage);
    }
}
