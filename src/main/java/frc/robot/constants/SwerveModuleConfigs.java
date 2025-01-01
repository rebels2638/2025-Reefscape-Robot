package frc.robot.constants;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// TODO: welcom go hell. this took way to long to make :======\
public final class SwerveModuleConfigs {
    public static class ModuleConfig {
      public final String kCAN_BUS_NAME = "canivore";

      // Steer motor config
      public final int kSTEER_CAN_ID;

      public static final double kSTEER_SUPPLY_CURRENT_LIMIT = 70;
      public static final double kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_TIME = 0.5;
      public static final double kSTEER_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT = 30;

      public static final double kSTEER_STATOR_CURRENT_LIMIT = 40;

      public static final double kSTEER_PEAK_FORWARD_TORQUE_CURRENT = 0.5;
      public static final double kSTEER_PEAK_REVERSE_TORQUE_CURRENT = 0.5;
      
      public static final double kSTEER_KS = 0.25;
      public static final double kSTEER_KV = 0.12;
      public static final double kSTEER_KA = 0.01;
      public static final double kSTEER_KP = 12;
      public static final double kSTEER_KI = 0;
      public static final double kSTEER_KD = 0;

      public static final double kSTEER_MOTION_MAGIC_EXPO_KA = 0;
      public static final double kSTEER_MOTION_MAGIC_EXPO_KV = 0;
      public static final double kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC = 0;

      public final boolean kSTEER_IS_INVERTED;
      public final boolean kSTEER_IS_NEUTRAL_MODE_BRAKE;

      public static final double kSTEER_MOTOR_TO_OUTPUT_SHAFT_RATIO = 1;
      public static final double kSTEER_ROTOR_TO_SENSOR_RATIO = 1;
      public static final boolean kSTEER_CONTINUOUS_WRAP = true;

      public static final FeedbackSensorSourceValue kSTEER_CANCODER_FEEDBACK_SENSOR_SOURCE = FeedbackSensorSourceValue.FusedCANcoder;
      
      // Drive motor config
      public final int kDRIVE_CAN_ID;

      public static final double kDRIVE_SUPPLY_CURRENT_LIMIT = 70;
      public static final double kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_TIME = 0.5;
      public static final double kDRIVE_SUPPLY_CURRENT_LIMIT_LOWER_LIMIT = 30;

      public static final double kDRIVE_STATOR_CURRENT_LIMIT = 40;

      public static final double kDRIVE_PEAK_FORWARD_TORQUE_CURRENT = 0.5;
      public static final double kDRIVE_PEAK_REVERSE_TORQUE_CURRENT = 0.5;
      
      public static final double kDRIVE_KS = 0.25;
      public static final double kDRIVE_KV = 0.12;
      public static final double kDRIVE_KA = 0.01;
      public static final double kDRIVE_KP = 12;
      public static final double kDRIVE_KI = 0;
      public static final double kDRIVE_KD = 0;

      public static final double kDRIVE_MOTION_MAGIC_VELOCITY_ACCELERATION_METERS_PER_SEC_SEC = 0;
      public static final double kDRIVE_MOTION_MAGIC_VELOCITY_JERK_METERS_PER_SEC_SEC_SEC = 0;
      public static final double kDRIVE_MOTION_MAGIC_CRUISE_VELOCITY = 0;

      public final boolean kDRIVE_IS_INVERTED;
      public final boolean kDRIVE_IS_NEUTRAL_MODE_BRAKE;

      public static final double kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO = 1;
      public static final double kDRIVE_WHEEL_RADIUS_METERS = 0.0762;
      public static final boolean kDRIVE_CONTINUOUS_WRAP = false;

      // CANCoder config
      public static final SensorDirectionValue kCANCODER_SENSOR_DIRECTION = SensorDirectionValue.CounterClockwise_Positive; 
      public static final int kCANCODER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 1;
      public final int kCANCODER_CAN_ID;
      public final double kCANCODER_OFFSET_ROTATIONS;
      
      public ModuleConfig(
        int kDRIVE_CAN_ID, 
        int kSTEER_CAN_ID, 
        boolean kDRIVE_IS_INVERTED, 
        boolean kSTEER_IS_INVERTED,
        boolean kDRIVE_IS_NEUTRAL_MODE_BRAKE,
        boolean kSTEER_IS_NEUTRAL_MODE_BRAKE,
        int kCANCODER_CAN_ID,
        double kCANCODER_OFFSET_ROTATIONS) {
          this.kDRIVE_CAN_ID = kDRIVE_CAN_ID;
          this.kSTEER_CAN_ID = kSTEER_CAN_ID;
          this.kDRIVE_IS_INVERTED = kDRIVE_IS_INVERTED;
          this.kSTEER_IS_INVERTED = kSTEER_IS_INVERTED;
          this.kDRIVE_IS_NEUTRAL_MODE_BRAKE = kDRIVE_IS_NEUTRAL_MODE_BRAKE;
          this.kSTEER_IS_NEUTRAL_MODE_BRAKE = kSTEER_IS_NEUTRAL_MODE_BRAKE;
          this.kCANCODER_CAN_ID = kCANCODER_CAN_ID;
          this.kCANCODER_OFFSET_ROTATIONS = kCANCODER_OFFSET_ROTATIONS;
      }
    }
     

    public static final ModuleConfig kFRONT_RIGHT_MODULE_CONFIG = new ModuleConfig(
      1, 2, false, false, false, false, 1, 0.0);
    public static final ModuleConfig kBACK_RIGHT_MODULE_CONFIG = new ModuleConfig(
      3, 4, false, false, false, false, 2, 0.0);
    public static final ModuleConfig kFRONT_LEFT_MODULE_CONFIG = new ModuleConfig(
      5, 6, false, false, false, false, 3, 0.0);
    public static final ModuleConfig kBACK_LEFT_MODULE_CONFIG = new ModuleConfig(
      7, 8, false, false, false, false, 4, 0.0);
    
}