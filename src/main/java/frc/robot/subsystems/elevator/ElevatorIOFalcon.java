package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.RebelUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOFalcon extends SubsystemBase implements ElevatorIO {
    private static final double kMotorToOutputShaftRatio = 1/7.5; 
    private static final double kSproketDiameterMeters = 0.032;
    private static final double kFIRST_STAGE_TO_SECOND = 2.054054054054054;
    private static final double kSECOND_STAGE_TO_THIRD = 1; //They got rid of the third stage

    private TalonFX m_motor1 = new TalonFX(17);
    private TalonFX m_motor2 = new TalonFX(18);
    // private static final double kMAX_CURRENT_AMPS = 35; //Let the smart current handler in the motorControllers handle it. 
    private static final double kMAX_VOLTAGE = 12;
    private static final double kELEVATOR_ANGLE_SIN = Math.sin(Math.toRadians(23));
    
    // Should be where the second stage starts when elavator down. measured from the base of the elevator to the second stage top in meters. do not measure from belly pan. measure from base of elavator
    
    // Should be where the second stage starts when elavator down. measured from the base of the elevator to the second stage top in meters. do not measure from belly pan. measure from base of elavator
    
    private static final double kMIN_SHOOTER_HEIGHT = 0; // 0.6m offset, stop the cap
    private static final double kMAX_SHOOTER_HEIGHT= 0.52;

    private static final double kMIN_CLIMBER_HEIGHT = 0;
    private static final double kMAX_CLIMBER_HEIGHT= 0.7;

    private double kCLIMB_KG = 0;
    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ElevatorFeedforward positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);
    private ElevatorFeedforward climbElevatorFeedforwardController = new ElevatorFeedforward(0,0,0);

    private double lastShooterHeightMeters;
    private double lastClimberHeightMeters;
    private double goalPositionMeters;


    public ElevatorIOFalcon() {

        m_motor1.clearStickyFault_BootDuringEnable();
        m_motor2.clearStickyFault_BootDuringEnable();
        m_motor1.setNeutralMode(NeutralModeValue.Brake);
        m_motor2.setNeutralMode(NeutralModeValue.Brake);

        zeroHeight();

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.shooterHeightMeters = m_motor1.getPosition().getValueAsDouble() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters * kFIRST_STAGE_TO_SECOND;

        inputs.climberHeightMeters = m_motor1.getPosition().getValueAsDouble() * kMotorToOutputShaftRatio * Math.PI *
                                                            kSproketDiameterMeters * kFIRST_STAGE_TO_SECOND * kSECOND_STAGE_TO_THIRD * kELEVATOR_ANGLE_SIN;

        inputs.voltageOut = m_motor1.getDutyCycle().getValueAsDouble() * kMAX_VOLTAGE;

        inputs.reachedSetpoint = positionFeedBackController.atSetpoint();
        
        inputs.goalPositionMeters = this.goalPositionMeters;

        lastClimberHeightMeters = inputs.climberHeightMeters;
        lastShooterHeightMeters = inputs.shooterHeightMeters;
    }

    @Override
    public void setHeightMeters(double goalPositionMeters) {
        positionFeedBackController.setSetpoint(goalPositionMeters);

        this.goalPositionMeters = goalPositionMeters;
        double currentPositionMeters = lastShooterHeightMeters;
        
        double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
        double accel = feedBackControllerVoltage == 0 ? 0 : feedBackControllerVoltage < 0 ? -1: 1; //Changes direction of accel given the feedbackcontroller voltage.
        double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, accel);

        double voltageOut = feedForwardVoltage + feedBackControllerVoltage;
        voltageOut = RebelUtil.constrain(voltageOut, -12, 12);

        if ((currentPositionMeters > kMAX_SHOOTER_HEIGHT && voltageOut > 0) || 
            (currentPositionMeters < kMIN_SHOOTER_HEIGHT && voltageOut < 0) || 
            (goalPositionMeters > kMAX_SHOOTER_HEIGHT || goalPositionMeters < kMIN_SHOOTER_HEIGHT)) {
            voltageOut = 0;
        }
        
        Logger.recordOutput("Elevator/voltageOut", voltageOut);
        m_motor1.setVoltage(voltageOut);
        m_motor2.setVoltage(voltageOut);

    }

    public void setVoltage(double voltage){
        Logger.recordOutput("Elevator/voltageOut", voltage);
        m_motor1.setVoltage(voltage);
        m_motor2.setVoltage(voltage);
    }
    

    @Override
    public void configureController(ElevatorFeedforward pff, PIDController pfb, ElevatorFeedforward Climbff,double kCLIMB_KG) {
        this.kCLIMB_KG = kCLIMB_KG;
        positionFeedBackController = pfb;
        positionFeedForwardController = pff;
        climbElevatorFeedforwardController = Climbff;
    }

    @Override
    public void zeroHeight() {
        m_motor1.setPosition(0.0);
        m_motor2.setPosition(0.0);
    }

}