package frc.robot.constants.swerve;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleConfig {
    public static class GeneralConfig {
      public final String kCAN_BUS_NAME;

      // Drive motor config
      public final double kDRIVE_SUPPLY_CURRENT_LIMIT;
      public final double kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_TIME;
      public final double kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT;

      public final double kDRIVE_STATOR_CURRENT_LIMIT;

      public final double kDRIVE_PEAK_FORWARD_TORQUE_CURRENT;
      public final double kDRIVE_PEAK_REVERSE_TORQUE_CURRENT;
      
      public final double kDRIVE_KS;
      public final double kDRIVE_KV;
      public final double kDRIVE_KA;
      public final double kDRIVE_KP;
      public final double kDRIVE_KI;
      public final double kDRIVE_KD;

      public final double kDRIVE_MAX_VELOCITY_METERS_PER_SEC;
      public final double kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC;
      public final double kDRIVE_MOTION_MAGIC_VELOCITY_JERK_METERS_PER_SEC_SEC_SEC;
      

      public final boolean kDRIVE_IS_NEUTRAL_MODE_BRAKE;

      public final double kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO;
      public final double kDRIVE_WHEEL_RADIUS_METERS;
      public final boolean kDRIVE_CONTINUOUS_WRAP;

      // Steer motor config
      public final double kSTEER_SUPPLY_CURRENT_LIMIT;
      public final double kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_TIME;
      public final double kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT;

      public final double kSTEER_STATOR_CURRENT_LIMIT;

      public final double kSTEER_PEAK_FORWARD_TORQUE_CURRENT;
      public final double kSTEER_PEAK_REVERSE_TORQUE_CURRENT;
      
      public final double kSTEER_KS;
      public final double kSTEER_KV;
      public final double kSTEER_KA;
      public final double kSTEER_KP;
      public final double kSTEER_KI;
      public final double kSTEER_KD;

      public final double kSTEER_MOTION_MAGIC_EXPO_KA;
      public final double kSTEER_MOTION_MAGIC_EXPO_KV;
      public final double kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC;

      public final boolean kSTEER_IS_NEUTRAL_MODE_BRAKE;

      public final double kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO;
      public final double kSTEER_ROTOR_TO_SENSOR_RATIO;
      public final boolean kSTEER_CONTINUOUS_WRAP;

      public final FeedbackSensorSourceValue kSTEER_CANCODER_FEEDBACK_SENSOR_SOURCE;

      // CANCoder config
      public final SensorDirectionValue kCANCODER_SENSOR_DIRECTION; 
      public final int kCANCODER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT;

      public GeneralConfig(
        String canBusName,
        double driveSupplyCurrentLimit,
        double driveSupplyCurrentLimitLowerTime,
        double driveSupplyCurrentLimitLowerLimit,
        double driveStatorCurrentLimit,
        double drivePeakForwardTorqueCurrent,
        double drivePeakReverseTorqueCurrent,
        double driveKS,
        double driveKV,
        double driveKA,
        double driveKP,
        double driveKI,
        double driveKD,
        double driveMotionMagicVelocityAccel,
        double driveMotionMagicVelocityJerk,
        double driveMaxVelocityMetersPerSec,
        boolean driveIsNeutralModeBrake,
        double driveMotorToOutputShaftRatio,
        double driveWheelRadiusMeters,
        boolean driveContinuousWrap,
        double steerSupplyCurrentLimit,
        double steerSupplyCurrentLimitLowerTime,
        double steerSupplyCurrentLimitLowerLimit,
        double steerStatorCurrentLimit,
        double steerPeakForwardTorqueCurrent,
        double steerPeakReverseTorqueCurrent,
        double steerKS,
        double steerKV,
        double steerKA,
        double steerKP,
        double steerKI,
        double steerKD,
        double steerMotionMagicExpoKA,
        double steerMotionMagicExpoKV,
        double steerMotionMagicCruiseVelRadPerSec,
        boolean steerIsNeutralModeBrake,
        double steerMotorToOutputShaftRatio,
        double steerRotorToSensorRatio,
        boolean steerContinuousWrap,
        FeedbackSensorSourceValue steerCancoderFeedbackSensorSource,
        SensorDirectionValue cancoderSensorDirection,
        int cancoderAbsSensorDiscontinuityPoint
      ) {
        this.kCAN_BUS_NAME = canBusName;
        this.kDRIVE_SUPPLY_CURRENT_LIMIT = driveSupplyCurrentLimit;
        this.kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_TIME = driveSupplyCurrentLimitLowerTime;
        this.kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT = driveSupplyCurrentLimitLowerLimit;
        this.kDRIVE_STATOR_CURRENT_LIMIT = driveStatorCurrentLimit;
        this.kDRIVE_PEAK_FORWARD_TORQUE_CURRENT = drivePeakForwardTorqueCurrent;
        this.kDRIVE_PEAK_REVERSE_TORQUE_CURRENT = drivePeakReverseTorqueCurrent;
        this.kDRIVE_KS = driveKS;
        this.kDRIVE_KV = driveKV;
        this.kDRIVE_KA = driveKA;
        this.kDRIVE_KP = driveKP;
        this.kDRIVE_KI = driveKI;
        this.kDRIVE_KD = driveKD;
        this.kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC = driveMotionMagicVelocityAccel;
        this.kDRIVE_MOTION_MAGIC_VELOCITY_JERK_METERS_PER_SEC_SEC_SEC = driveMotionMagicVelocityJerk;
        this.kDRIVE_MAX_VELOCITY_METERS_PER_SEC = driveMaxVelocityMetersPerSec;
        this.kDRIVE_IS_NEUTRAL_MODE_BRAKE = driveIsNeutralModeBrake;
        this.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO = driveMotorToOutputShaftRatio;
        this.kDRIVE_WHEEL_RADIUS_METERS = driveWheelRadiusMeters;
        this.kDRIVE_CONTINUOUS_WRAP = driveContinuousWrap;
        this.kSTEER_SUPPLY_CURRENT_LIMIT = steerSupplyCurrentLimit;
        this.kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_TIME = steerSupplyCurrentLimitLowerTime;
        this.kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT = steerSupplyCurrentLimitLowerLimit;
        this.kSTEER_STATOR_CURRENT_LIMIT = steerStatorCurrentLimit;
        this.kSTEER_PEAK_FORWARD_TORQUE_CURRENT = steerPeakForwardTorqueCurrent;
        this.kSTEER_PEAK_REVERSE_TORQUE_CURRENT = steerPeakReverseTorqueCurrent;
        this.kSTEER_KS = steerKS;
        this.kSTEER_KV = steerKV;
        this.kSTEER_KA = steerKA;
        this.kSTEER_KP = steerKP;
        this.kSTEER_KI = steerKI;
        this.kSTEER_KD = steerKD;
        this.kSTEER_MOTION_MAGIC_EXPO_KA = steerMotionMagicExpoKA;
        this.kSTEER_MOTION_MAGIC_EXPO_KV = steerMotionMagicExpoKV;
        this.kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC = steerMotionMagicCruiseVelRadPerSec;
        this.kSTEER_IS_NEUTRAL_MODE_BRAKE = steerIsNeutralModeBrake;
        this.kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO = steerMotorToOutputShaftRatio;
        this.kSTEER_ROTOR_TO_SENSOR_RATIO = steerRotorToSensorRatio;
        this.kSTEER_CONTINUOUS_WRAP = steerContinuousWrap;
        this.kSTEER_CANCODER_FEEDBACK_SENSOR_SOURCE = steerCancoderFeedbackSensorSource;
        this.kCANCODER_SENSOR_DIRECTION = cancoderSensorDirection;
        this.kCANCODER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = cancoderAbsSensorDiscontinuityPoint;
      }
    }

    public static class SpecificConfig {
      public final int kDRIVE_CAN_ID;
      public final int kSTEER_CAN_ID;

      public final boolean kDRIVE_IS_INVERTED;
      public final boolean kSTEER_IS_INVERTED;

      public final int kCANCODER_CAN_ID;
      public final double kCANCODER_OFFSET_ROTATIONS;

      public SpecificConfig(
        int kDRIVE_CAN_ID, 
        int kSTEER_CAN_ID, 
        boolean kDRIVE_IS_INVERTED, 
        boolean kSTEER_IS_INVERTED,
        int kCANCODER_CAN_ID,
        double kCANCODER_OFFSET_ROTATIONS) {
          this.kDRIVE_CAN_ID = kDRIVE_CAN_ID;
          this.kSTEER_CAN_ID = kSTEER_CAN_ID;
          this.kDRIVE_IS_INVERTED = kDRIVE_IS_INVERTED;
          this.kSTEER_IS_INVERTED = kSTEER_IS_INVERTED;
          this.kCANCODER_CAN_ID = kCANCODER_CAN_ID;
          this.kCANCODER_OFFSET_ROTATIONS = kCANCODER_OFFSET_ROTATIONS;
      }
    }

    public final GeneralConfig kGENERAL_CONFIG;
    public final SpecificConfig kSPECIFIC_CONFIG;
    public SwerveModuleConfig(
      int kDRIVE_CAN_ID, 
      int kSTEER_CAN_ID, 
      boolean kDRIVE_IS_INVERTED, 
      boolean kSTEER_IS_INVERTED,
      boolean kDRIVE_IS_NEUTRAL_MODE_BRAKE,
      boolean kSTEER_IS_NEUTRAL_MODE_BRAKE,
      int kCANCODER_CAN_ID,
      double kCANCODER_OFFSET_ROTATIONS,
      GeneralConfig kGENERAL_CONFIG) {
        this.kGENERAL_CONFIG = kGENERAL_CONFIG;
        this.kSPECIFIC_CONFIG = new SpecificConfig(
          kDRIVE_CAN_ID, 
          kSTEER_CAN_ID, 
          kDRIVE_IS_INVERTED, 
          kSTEER_IS_INVERTED,
          kCANCODER_CAN_ID,
          kCANCODER_OFFSET_ROTATIONS);
    }
  }