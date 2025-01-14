package frc.robot.subsystems.drivetrain.swerve;


import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class GyroIONavX implements GyroIO {
    private AHRS gyro;
    private Rotation3d offset = new Rotation3d();
    
    public GyroIONavX() {
        try {
            gyro = new AHRS(NavXComType.kMXP_SPI);
        }
        catch (RuntimeException ex) {
            System.err.println("Error initilizing Gyro");
        }
    }

    @Override
    public void setOffset(Rotation3d offset) {
        this.offset = offset;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = gyro.isConnected();
        if (inputs.isConnected) {
            inputs.yaw = new Rotation2d(-(Math.toRadians(gyro.getYaw()) - offset.getZ()));
            inputs.yawRadSec = -gyro.getRate();
        }
    }

    @Override
    public void zero() {
        gyro.zeroYaw();
    }

    @Override
    public void reset(Rotation3d inital) {
        offset = gyro.getRotation3d().plus(inital);
    }

}
