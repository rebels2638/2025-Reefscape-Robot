package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.climber.ClimberConfigBase;
import frc.robot.constants.climber.ClimberConfigComp;
import frc.robot.constants.climber.ClimberConfigProto;
import frc.robot.constants.climber.ClimberConfigSim;

public class Climber extends SubsystemBase {
    private static Climber instance = null;
    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();

    private Rotation2d setpoint = Rotation2d.fromDegrees(205);

    private final ClimberConfigBase config;

    private Climber() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = ClimberConfigComp.getInstance();
                climberIO = new ClimberIOTalonFX(config);

                break;

            case PROTO:
                config = ClimberConfigProto.getInstance();
                climberIO = new ClimberIOTalonFX(config);

                break;

            case SIM:
                config = ClimberConfigSim.getInstance();
                climberIO = new ClimberIOSim(config);

                break;

            case REPLAY:
                config = ClimberConfigComp.getInstance();
                climberIO = new ClimberIO() {};

                break;

            default:
                config = ClimberConfigComp.getInstance();
                climberIO = new ClimberIOTalonFX(config);

                break;
        }

        setpoint = Rotation2d.fromRotations(config.getStartingAngleRotations());

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberIOInputs);
        Logger.processInputs("climber", climberIOInputs);

        climberIO.setAngle(setpoint);
    }

    public void setAngle(Rotation2d angle) {
        setpoint = angle;
        Logger.recordOutput("climber/setpoint", angle);
    }

    public void setTorqueCurrentFOC(double torque) {
        climberIO.setTorqueCurrentFOC(torque);
    }

    public void setVoltage(double voltage) {
        climberIO.setVoltage(voltage);
    }

    public boolean reachedSetpoint() {
        return Math.abs(setpoint.minus(climberIOInputs.climberPosition).getDegrees()) <= config.getToleranceDegrees();
    }

    public Rotation2d getAngle() {
        return climberIOInputs.climberPosition;
    }

    public Rotation2d getSetpoint() {
        return setpoint;
    }
}
