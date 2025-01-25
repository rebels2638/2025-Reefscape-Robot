// package frc.robot.subsystems.roller;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import frc.robot.constants.pivot.PivotConfigBase;
// import frc.robot.constants.roller.RollerConfigBase;

// public class RollerIOSim implements RollerIO {
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
//     public RollerIOSim(RollerConfigBase config) {
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

//         currentProfileSetpoint = new State(pivotSim.getAngleRads(), pivotSim.getVelocityRadPerSec());
//     }

//     @Override
//     public void updateInputs(RollerIOInputs inputs) {
//         double dt = Timer.getFPGATimestamp() - prevTimeInputs;
//         pivotSim.update(dt);

//         inputs.rollerVelocityRadPerSec = pivotSim.getVelocityRadPerSec();

//         inputs.rollerAppliedVolts = appliedVolts;

//         prevTimeInputs = Timer.getFPGATimestamp();
//     }

//     @Override
//     public void setTorqueCurrentFOC(double current) {
//         appliedVolts = current;
//         pivotSim.setInputVoltage(current);

//     }
// }
