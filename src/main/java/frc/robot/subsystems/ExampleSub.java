// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ExampleSub extends SubsystemBase {
//     private DCMotor gearbox = DCMotor.getNeo550(1);
//     private FlywheelSim motor = new FlywheelSim(gearbox, 1, 0.007);

//     private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
//     private final PIDController pidController = new PIDController(0, 0, 0);

//     private double desiredSpeed = 0;

//     @Override
//     public void periodic() {
//         double voltage = feedforward.calculate(desiredSpeed) + pidController.calculate(getRpm(), desiredSpeed);
//         motor.setInputVoltage(voltage);

//         motor.update(0.020);
//     }

//     public double getRpm() {
//         return motor.getAngularVelocityRPM();
//     }

//     public void setDesiredSpeed(double s) {
//         desiredSpeed = s;
//     }

// }