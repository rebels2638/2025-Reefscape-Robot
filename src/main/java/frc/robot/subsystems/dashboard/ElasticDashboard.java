package frc.robot.subsystems.dashboard;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.autoAligment.AutoAlign;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class ElasticDashboard extends SubsystemBase{
    private static Field2d field = new Field2d();

    private final DecimalFormat format = new DecimalFormat("0.00");

    private final SwerveDrive swerveDrive;

    private final AutoAlign autoAlignment;

    public ElasticDashboard (SwerveDrive swerveDrive, AutoAlign autoAlign) {
        this.swerveDrive = swerveDrive;
        this.autoAlignment = autoAlign;
    }

    //very bummy but idc
    @Override
    public void periodic() {
        Pose2d pose = swerveDrive.getPose();
        String x = format.format(pose.getX());
        String y = format.format(pose.getY());
        String rotation = format.format(pose.getRotation().getRadians());
        field.setRobotPose(pose);
        SmartDashboard.putData("Dashboard/Field", field);
        SmartDashboard.putStringArray("Dashboard/EstimatedPose", new String[]{x, y, rotation});

        boolean alignmentIsFinished = autoAlignment.isFinished();
        SmartDashboard.putBoolean("Dashboard/AutoAlignmentStatus", alignmentIsFinished);

        SmartDashboard.putNumber("Dashboard/MatchTime", DriverStation.getMatchTime());
    }
}