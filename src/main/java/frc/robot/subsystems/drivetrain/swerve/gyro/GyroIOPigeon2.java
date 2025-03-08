package frc.robot.subsystems.drivetrain.swerve.gyro;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.util.Elastic;
import frc.robot.subsystems.drivetrain.swerve.Phoenix6Odometry;
import frc.robot.lib.util.PhoenixUtil;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 gyro;

    private final StatusSignal<Angle> yawSignal;
    private final StatusSignal<AngularVelocity> yawVelocitySignal;

    private final StatusSignal<Angle> rollSignal;
    private final StatusSignal<AngularVelocity> rollVelocitySignal;

    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<AngularVelocity> pitchVelocitySignal;

    private final StatusSignal<LinearAcceleration> accelerationXSignal;
    private final StatusSignal<LinearAcceleration> accelerationYSignal;

    private final Phoenix6Odometry odom;

    private final Elastic.Notification gyroDisconnectAlert = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
                                "Gyro Disconnected", "Pigeon2 Disconnected, Gyro may not be functioning");


    private final Debouncer connectedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

    public GyroIOPigeon2() {
        odom = Phoenix6Odometry.getInstance();
        gyro = new Pigeon2(13, "drivetrain");
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = 179.86917114257812;
        config.MountPose.MountPosePitch = -2.2904083728790283;
        config.MountPose.MountPoseRoll = 0.06304000318050385;
        config.GyroTrim.GyroScalarZ = 0;

        PhoenixUtil.tryUntilOk(5, () -> gyro.getConfigurator().apply(config, 0.25));

        yawSignal = gyro.getYaw().clone();
        yawVelocitySignal = gyro.getAngularVelocityZWorld().clone();

        rollSignal = gyro.getRoll().clone();
        rollVelocitySignal = gyro.getAngularVelocityXWorld().clone();

        pitchSignal = gyro.getPitch().clone();
        pitchVelocitySignal = gyro.getAngularVelocityYWorld().clone();

        accelerationXSignal = gyro.getAccelerationX().clone();
        accelerationYSignal = gyro.getAccelerationY().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                70,
                yawSignal,
                yawVelocitySignal,
                rollSignal,
                rollVelocitySignal,
                pitchSignal,
                pitchVelocitySignal,
                accelerationXSignal,
                accelerationYSignal);

        odom.registerSignal(gyro, yawSignal);
        odom.registerSignal(gyro, yawVelocitySignal);
        odom.registerSignal(gyro, rollSignal);
        odom.registerSignal(gyro, rollVelocitySignal);
        odom.registerSignal(gyro, pitchSignal);
        odom.registerSignal(gyro, pitchVelocitySignal);
        odom.registerSignal(gyro, accelerationXSignal);
        odom.registerSignal(gyro, accelerationYSignal);

        gyro.optimizeBusUtilization();
    }

    @Override
    public synchronized void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = 
            connectedDebouncer.calculate(
                BaseStatusSignal.
                    refreshAll(
                        yawSignal,
                        yawVelocitySignal,
                        rollSignal,
                        rollVelocitySignal,
                        pitchSignal,
                        pitchVelocitySignal,
                        accelerationXSignal,
                        accelerationYSignal
                    ).isOK()
            );
            

        // TODO: CHECK FOR FEILD VS GYRO RELATIVE VALUES
        inputs.orientation = new Rotation2d[] {
            new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(rollSignal, rollVelocitySignal).in(Radians)),
            new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(pitchSignal, pitchVelocitySignal).in(Radians)),
            new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(yawSignal, yawVelocitySignal).in(Radians))
        };
        inputs.rates = new Rotation2d[] {
            new Rotation2d(rollVelocitySignal.getValue().in(RadiansPerSecond)),
            new Rotation2d(pitchVelocitySignal.getValue().in(RadiansPerSecond)),
            new Rotation2d(yawVelocitySignal.getValue().in(RadiansPerSecond))
        };

        Logger.recordOutput("SKIBIDIFUCK", inputs.rates[2]);

        inputs.fieldRelativeAccelerationMetersPerSecSec = new Translation2d(
            accelerationXSignal.getValue().in(MetersPerSecondPerSecond), 
            accelerationYSignal.getValue().in(MetersPerSecondPerSecond)
        );

        if (!inputs.isConnected) {
            Elastic.sendNotification(gyroDisconnectAlert.withDisplayMilliseconds(10000));
            DriverStation.reportError("Roller CANRange Disconnected", true);
        }
    }

    @Override
    public void resetGyro(Rotation2d yaw) {
        gyro.setYaw(yaw.getDegrees());
    }
}