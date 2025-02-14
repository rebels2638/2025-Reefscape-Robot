package frc.robot.subsystems.drivetrain.swerve.gyro;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

public class GyroIONavX implements GyroIO {
    private AHRS gyro;
    private Rotation2d yawOffset = new Rotation2d(0);
    
    public GyroIONavX() {
        try {
            gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        }
        catch (RuntimeException ex) {
            System.err.println("Error initializing Gyro");
        }
    }


    @Override
    public synchronized void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = gyro.isConnected();
        if (inputs.isConnected) {
            inputs.orientation = new Rotation3d(
                Math.toRadians(-gyro.getRoll()) + yawOffset.getRadians(),
                Math.toRadians(-gyro.getPitch()),
                Math.toRadians(-gyro.getYaw())
            );

            inputs.rates = new Rotation3d(
                0,
                0,
                Math.toRadians(-gyro.getRate())
            );

            inputs.fieldRelativeAccelerationMetersPerSecSec = new Translation2d(
                gyro.getWorldLinearAccelX(),
                gyro.getWorldLinearAccelY()
            );
        }
    }

    @Override 
    public void resetGyro(Rotation2d yaw) {
        gyro.zeroYaw();
        yawOffset = yaw;
    }

}
