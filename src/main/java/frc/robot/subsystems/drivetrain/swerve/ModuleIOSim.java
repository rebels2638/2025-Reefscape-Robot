// package frc.robot.subsystems.drivetrain.swerve;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.signals.MagnetHealthValue;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import frc.robot.Constants;
// import frc.robot.lib.util.RebelUtil;

// public class ModuleIOSim implements ModuleIO {
//     private DCMotor m_gearBoxAngle = DCMotor.getFalcon500Foc(1);
//     private DCMotor m_gearBoxDrive = DCMotor.getFalcon500Foc(1);

//     private FlywheelSim m_angleSim = new FlywheelSim(m_gearBoxAngle, 1, 0.0001);
//     private FlywheelSim m_driveSim = new FlywheelSim(m_gearBoxDrive, 1, 0.003);
//     private static final PIDController m_angleFeedbackController = new PIDController(0.06, 0.0, 0.0002);
//     private static final PIDController m_driveFeedbackController = new PIDController(0.00, 0, 0);

//     private static final SimpleMotorFeedforward m_angleFeedforward = new SimpleMotorFeedforward(0, 0.0000);
//     private static final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0.0063, 0.001);
//     private double m_angleVoltage = 0;
//     private double m_driveVoltage = 0;
//     private double anglePositionRad = 0;
//     private double drivePositionsMeters = 0;

//     private double prevTimeIO = 0;
//     private double prevTime = 0;

//     private double driveVelocityMps = 0;

//     public ModuleIOSim() {
//         m_angleFeedbackController.setTolerance(Math.toRadians(3));
//         m_driveFeedbackController.setTolerance(0.01);

//         m_angleFeedbackController.enableContinuousInput(0, 2 * Math.PI);
//     }
//     @Override
//     public void updateInputs(ModuleIOInputs inputs) {
//         m_angleSim.update(Timer.getFPGATimestamp() - prevTimeIO);
//         m_driveSim.update(Timer.getFPGATimestamp() - prevTimeIO);

//         inputs.angleVelocityRadPerSec = m_angleSim.getAngularVelocityRPM();
//         inputs.driveVelocityMps = m_driveSim.getAngularVelocityRPM() * Constants.DrivetrainConstants.kDRIVE_WHEEL_RADIUS_METERS * 2 * Math.PI;
//         this.driveVelocityMps = inputs.driveVelocityMps;
//         anglePositionRad += inputs.angleVelocityRadPerSec * (Timer.getFPGATimestamp() - prevTimeIO);
//         drivePositionsMeters += inputs.driveVelocityMps * (Timer.getFPGATimestamp() - prevTimeIO);

//         anglePositionRad = anglePositionRad % (2 * Math.PI);

//         inputs.anglePositionRad = anglePositionRad;
//         inputs.drivePositionMeters = drivePositionsMeters;

//         inputs.angleCurrentAmps = m_angleSim.getCurrentDrawAmps();
//         inputs.driveCurrentAmps = m_driveSim.getCurrentDrawAmps();

//         inputs.angleTempC = 0;
//         inputs.driveTempC = 0;

//         inputs.angleVoltage = m_angleVoltage;
//         inputs.driveVoltage = m_driveVoltage;

//         prevTimeIO = Timer.getFPGATimestamp();
//     }

//     @Override
//     public void setState(SwerveModuleState state) {
//         if (DriverStation.isTest()) {return;}

//         double speed = state.speedMetersPerSecond;
//         m_driveVoltage = m_driveFeedbackController.calculate(driveVelocityMps, speed);
//         m_driveVoltage += m_driveFeedforward.calculate(
//         driveVelocityMps,
//         state.speedMetersPerSecond,
//         (Timer.getFPGATimestamp() - prevTime));

//         m_driveVoltage = RebelUtil.constrain(
//         m_driveVoltage, -Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE,
//         Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE);

//         m_driveSim.setInputVoltage(m_driveVoltage);

//         double desiredAngle = state.angle.getRadians();

//         m_angleVoltage = m_angleFeedbackController.calculate(anglePositionRad, desiredAngle);
//         double deltaTheta = desiredAngle - anglePositionRad;
//         if (deltaTheta < -Math.PI) {
//         deltaTheta = Math.PI * 2 + deltaTheta;
//         }
//         else if (deltaTheta > Math.PI) {
//         deltaTheta = Math.PI * 2 - deltaTheta;
//         }
//         Logger.recordOutput("SwerveDrive/deltaTheta", deltaTheta);
//         double velocityRadSec = (deltaTheta) / (Timer.getFPGATimestamp() - prevTime);
//         Logger.recordOutput("SwerveDrive/velocityRadSec", velocityRadSec);
//         m_angleVoltage += m_angleFeedforward.calculate(velocityRadSec, Math.signum(m_angleVoltage)); // TODO: this is scuffed
//         m_angleVoltage = RebelUtil.constrain(
//         m_angleVoltage, -Constants.DrivetrainConstants.kMAX_ANGLE_VOLTAGE,
//         Constants.DrivetrainConstants.kMAX_ANGLE_VOLTAGE);

//         m_angleSim.setInputVoltage(m_angleVoltage);

//         prevTime = Timer.getFPGATimestamp();
//     }

//     @Override
//     public void setDriveVoltage(double voltage) {
//         m_driveVoltage = RebelUtil.constrain(
//         voltage, -Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE,
//         Constants.DrivetrainConstants.kMAX_DRIVE_VOLTAGE);
//         m_driveSim.setInputVoltage(m_driveVoltage);
//     }

//     @Override
//     public void resetPosition() {
//         drivePositionsMeters = 0;
//     }
// }
