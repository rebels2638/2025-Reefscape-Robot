// package frc.robot.subsystems.shooter.roller;

// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// public class RollerIOSim implements RollerIO {
    
//     private static final double kMOTOR_TO_OUTPUT_RATIO = 2;

//     private DCMotor m_gearBoxT = DCMotor.getNeo550(1);
//     private DCMotor m_gearboxB = DCMotor.getNeo550(1);

//     private FlywheelSim m_topSim = new FlywheelSim(m_gearBoxT, kMOTOR_TO_OUTPUT_RATIO, 0.007);
//     private FlywheelSim m_bottomSim = new FlywheelSim(m_gearboxB, kMOTOR_TO_OUTPUT_RATIO, 0.007);

//     private double voltage = 0;

//     @Override
//     public void updateInputs(RollerIOInputs inputs) {
//         m_topSim.update(0.020);
//         m_bottomSim.update(0.020);

//         inputs.RPM = m_topSim.getAngularVelocityRPM();

//         inputs.tAmps = m_topSim.getCurrentDrawAmps();
//         inputs.bAmps = m_bottomSim.getCurrentDrawAmps();

//         inputs.tVolts = voltage;
//         inputs.bVolts = voltage;
//     }

//     public void setVoltage(double voltage) {
//         this.voltage = voltage;
//         m_topSim.setInputVoltage(voltage);
//         m_bottomSim.setInputVoltage(voltage);
//     }
// }
