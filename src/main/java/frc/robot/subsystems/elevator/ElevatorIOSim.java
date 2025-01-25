// package frc.robot.subsystems.elevator;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import frc.robot.constants.elevator.ElevatorConfigBase;
// import frc.robot.constants.pivot.PivotConfigBase;

// public class ElevatorIOSim implements ElevatorIO {
//     private DCMotor pivotGearBox = DCMotor.getKrakenX60Foc(1);

//     private SingleJointedArmSim pivotSim;

//     private final PIDController feedbackController = new PIDController(0, 0, 0);
//     private final ArmFeedforward feedforwardController = new ArmFeedforward(0, 0, 0);

//     private double prevTimeInputs = 0;
//     private double prevTimeState = 0;

//     private final double kPIVOT_MOTOR_TO_OUTPUT_SHAFT_RATIO = 20;
//     private final double kJKG_METERS_SQUARED = 0.0008096955;
//     private final double kPIVOT_LENGTH_METERS = 0.2;
//     private final double kMIN_ANGLE_RAD = Math.toRadians(-40);
//     private final double kMAX_ANGLE_RAD = Math.toRadians(100);
//     private final double kSTARTING_ANGLE_RAD = Math.toRadians(100);


//     private final TrapezoidProfile trapezoidMotionProfile;
//     private State currentProfileSetpoint;;

//     private double currentPositionRad = 0;
//     private double appliedVolts = 0;
    
//     @SuppressWarnings("static-access")
//     public ElevatorIOSim(ElevatorConfigBase config) {
//         pivotSim = new SingleJointedArmSim(
//             pivotGearBox,
//             kPIVOT_MOTOR_TO_OUTPUT_SHAFT_RATIO,
//             kJKG_METERS_SQUARED,
//             kPIVOT_LENGTH_METERS,
//             kMIN_ANGLE_RAD,
//             kMAX_ANGLE_RAD,
//             true,
//             kSTARTING_ANGLE_RAD
//         );

//         feedbackController.setTolerance(Math.toRadians(1.4));

//         trapezoidMotionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
//                 Units.rotationsToRadians(config.getMotionMagicCruiseVelocityRotationsPerSec()),
//                 12 / config.getMotionMagicExpoKA() // divide supply voltage to get max acell
//         ));
//         currentProfileSetpoint = new State(pivotSim.getAngleRads(), pivotSim.getVelocityRadPerSec());
//     }

//     @Override
//     public void updateInputs(ElevatorIOInputs inputs) {
//         double dt = Timer.getFPGATimestamp() - prevTimeInputs;
//         pivotSim.update(dt);

//         inputs.elevatorVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
//         inputs.elevatorPositionRad = new Rotation2d(pivotSim.getAngleRads());
//         this.currentPositionRad = inputs.elevatorPositionRad.getRadians();

//         inputs.elevatorAppliedVolts = appliedVolts;

//         prevTimeInputs = Timer.getFPGATimestamp();
//     }

//     @Override
//     public void setHeight(double angle) {
//         double dt = Timer.getFPGATimestamp() - prevTimeState;
        
//         currentProfileSetpoint = trapezoidMotionProfile.calculate(
//             dt, 
//             currentProfileSetpoint,
//             new State(
//                 angle,
//                 0
//             )
//         );

//         double voltage = 
//             feedforwardController.calculate(currentProfileSetpoint.position, currentProfileSetpoint.velocity) +
//             feedbackController.calculate(currentPositionRad, currentProfileSetpoint.position);
//         appliedVolts = voltage;

//         pivotSim.setInputVoltage(voltage);

//         prevTimeState = Timer.getFPGATimestamp();
//     }

//     @Override
//     public void setTorqueCurrentFOC(double voltage) {
//         appliedVolts = voltage;
//         pivotSim.setInputVoltage(voltage);

//     }
// }
