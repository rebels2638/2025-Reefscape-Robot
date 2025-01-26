package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class ElasticDashboard extends SubsystemBase{
    private static Field2d field = new Field2d();

    private final SwerveDrive swerveDrive;
    private final int i = 0;

    public ElasticDashboard (SwerveDrive s) {
        swerveDrive = s;
    }

    //very bummy but idc
    @Override
    public void periodic() {
        Pose2d pose = swerveDrive.getPose();
        double x = pose.getX();
        double y = pose.getY();
        double rotation = pose.getRotation().getRadians();
        field.setRobotPose(pose);
        SmartDashboard.putData("Dashboard/Field", field);
        SmartDashboard.putNumberArray("Dashboard/EstimatedPose", new double[]{x, y, rotation});
        SmartDashboard.putNumber("Dashboard/MatchTime", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Dashboard/Random", (double) i);
    }
}