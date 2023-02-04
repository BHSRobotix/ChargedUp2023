// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * @since 1/9/2023
 */
public final class Constants {
  /**
   * P value for the PID controller for the balance command. TODO: Change this to work with the
   * drive PID values
   */
  public static final double BALANCE_P = 0.39941;
  /**
   * I value for the PID controller for the balance command. TODO: Change this to work with the
   * drive PID values
   */
  public static final double BALANCE_I = 0.0;
  /**
   * D value for the PID controller for the balance command. TODO: Change this to work with the
   * drive PID values
   */
  public static final double BALANCE_D = 0.00;

  /**
   * Operator constants are for the operator interface (joysticks, buttons, etc.).
   *
   * @since 1/11/2023
   */
  public static class OperatorConstants {
    /**
     * The port for the joystick on the driver station. This is the main joystick and is used for
     * driving
     */
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  /**
   * Drive constants are for the drive train subsystem characteristics.
   *
   * @since 1/26/2023
   */
  public static class DriveConstants {
<<<<<<< HEAD
    /**
     * The deadband for the joystick. This is the minimum value that the joystick must be at to move
     * the robot.
     */
    public static final double JOYSTICK_DEADBAND = 0.035;
||||||| parent of cb90389 (messing around with sim pathplanning)
    /** The maximum acceleration of the robot in units per second */
    public static final double SLEW_RATE_LIMITER = 3;
    /** The maximum speed of the robot in meters per second */
    public static final double MAX_SPEED = 2.447; // meters per second TODO: Measure/SysID
    /** The maximum angular speed of the robot in radians per second */
    public static final double MAX_ANGULAR_SPEED =
        2 * Math.PI; // one rotation per second TODO: Measure

    /** The track width of the robot in meters */
    public static final double TRACK_WIDTH = 0.555; // meters, robot width TODO: Measure
    /** The wheel radius of the robot in meters */
    public static final double WHEEL_RADIUS =
        0.0762; // meters (Andymark am-0940b "High Grip Wheels, 6")
    /** The resolution of the encoder in counts per revolution */
    public static final int ENCODER_RESOLUTION = 4096; // CTRE Magnetic Encoder
    /** The gear ratio of the encoder to the drive shaft */
    public static final int ENCODER_GEAR_RATIO = 1; // Encoder is connected directly to drive shaft

    public static final class SysId {
      // Based on sysid_data20230127-205712.json TODO: Tune

      /** The P value for the PID controller for the drive train (velocity). Gotten from SYSID */
      public static final double FEED_BACK_VELOCITY_P = 3.5725;
      /** The I value for the PID controller for the drive train (velocity). Gotten from SYSID */
      public static final double FEED_BACK_VELOCITY_I = 0.0;
      /** The D value for the PID controller for the drive train (velocity). Gotten from SYSID */
      public static final double FEED_BACK_VELOCITY_D = 0.0;

      /** The P value for the PID controller for the drive train (position). Gotten from SYSID */
      public static final double FEED_BACK_POSITION_P = 94.989;
      /** The I value for the PID controller for the drive train (position). Gotten from SYSID */
      public static final double FEED_BACK_POSITION_I = 0.0;
      /** The D value for the PID controller for the drive train (position). Gotten from SYSID */
      public static final double FEED_BACK_POSITION_D = 8.7272;

      // Feedforward constants (straight-line) TODO: Tune
      /**
       * The S value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the static friction of the robot in a straight line.
       */
      public static final double FEED_FORWARD_LINEAR_S = 0.94143;
      /**
       * The V value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the velocity gain of the robot in a straight line.
       */
      public static final double FEED_FORWARD_LINEAR_V = 2.3803;
      /**
       * The A value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the acceleration gain of the robot in a straight line.
       */
      public static final double FEED_FORWARD_LINEAR_A = 0.48128;

      // Feedforward constants (rotating) TODO: Tune
      /**
       * The S value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the static friction of the robot when rotating.
       */
      public static final double FEED_FORWARD_ANGULAR_S = 1.5;
      /**
       * The V value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the velocity gain of the robot when rotating.
       */
      public static final double FEED_FORWARD_ANGULAR_V = 1.5;
      /**
       * The A value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the acceleration gain of the robot when rotating.
       */
      public static final double FEED_FORWARD_ANGULAR_A = 0.3;
=======
    /** The maximum acceleration of the robot in units per second */
    public static final double SLEW_RATE_LIMITER = 3;
    /** The maximum speed of the robot in meters per second */
    public static final double MAX_SPEED = 2.447; // meters per second TODO: Measure/SysID
    /** The maximum angular speed of the robot in radians per second */
    public static final double MAX_ANGULAR_SPEED =
        2 * Math.PI; // one rotation per second TODO: Measure

    /** The track width of the robot in meters */
    public static final double TRACK_WIDTH = 0.555; // meters, robot width TODO: Measure
    /** The wheel radius of the robot in meters */
    public static final double WHEEL_RADIUS =
        0.0762; // meters (Andymark am-0940b "High Grip Wheels, 6")

    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    /** The resolution of the encoder in counts per revolution */
    public static final int ENCODER_RESOLUTION = 4096; // CTRE Magnetic Encoder
    /** The gear ratio of the encoder to the drive shaft */
    public static final int ENCODER_GEAR_RATIO = 1; // Encoder is connected directly to drive shaft

    public static final class SysId {
      // Based on sysid_data20230127-205712.json TODO: Tune

      /** The P value for the PID controller for the drive train (velocity). Gotten from SYSID */
      public static final double FEED_BACK_VELOCITY_P = 0;//3.5725;
      /** The I value for the PID controller for the drive train (velocity). Gotten from SYSID */
      public static final double FEED_BACK_VELOCITY_I = 0.0;
      /** The D value for the PID controller for the drive train (velocity). Gotten from SYSID */
      public static final double FEED_BACK_VELOCITY_D = 0.0;

      /** The P value for the PID controller for the drive train (position). Gotten from SYSID */
      public static final double FEED_BACK_POSITION_P = 94.989;
      /** The I value for the PID controller for the drive train (position). Gotten from SYSID */
      public static final double FEED_BACK_POSITION_I = 0.0;
      /** The D value for the PID controller for the drive train (position). Gotten from SYSID */
      public static final double FEED_BACK_POSITION_D = 8.7272;

      // Feedforward constants (straight-line) TODO: Tune
      /**
       * The S value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the static friction of the robot in a straight line.
       */
      public static final double FEED_FORWARD_LINEAR_S = 0.94143;
      /**
       * The V value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the velocity gain of the robot in a straight line.
       */
      public static final double FEED_FORWARD_LINEAR_V = 2.3803;
      /**
       * The A value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the acceleration gain of the robot in a straight line.
       */
      public static final double FEED_FORWARD_LINEAR_A = 0.48128;

      // Feedforward constants (rotating) TODO: Tune
      /**
       * The S value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the static friction of the robot when rotating.
       */
      public static final double FEED_FORWARD_ANGULAR_S = 1.5;
      /**
       * The V value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the velocity gain of the robot when rotating.
       */
      public static final double FEED_FORWARD_ANGULAR_V = 1.5;
      /**
       * The A value for the feedforward controller for the drive train. Gotten from SYSID This is
       * the acceleration gain of the robot when rotating.
       */
      public static final double FEED_FORWARD_ANGULAR_A = 0.3;
>>>>>>> cb90389 (messing around with sim pathplanning)

    /** The maximum acceleration of the robot in units per second */
    public static final double SLEW_RATE_LIMITER = 2.5;

    public static final DCMotor MOTOR_CONFIGURATION = DCMotor.getCIM(2);
  }

  /**
   * Gripper constants are for the gripper subsystem
   *
   * @since 1/30/2023
   */
  public static class GripperConstants {
    /** The gripper double solenoid forward channel */
    public static final int GRIPPER_SOLENOID_FORWARD = 0;
    /** The gripper double solenoid reverse channel */
    public static final int GRIPPER_SOLENOID_REVERSE = 1;
    /** The CAN ID for the gripper's pneumatic compressor */
    public static final int COMPRESSOR_CAN_ID = 10;
  }

  public static final class SysIdConstants {
    /** Create a linear system from our system identification gains. */
    public static final LinearSystem<N2, N2, N2> PLANT =
        LinearSystemId.identifyDrivetrainSystem(
            Robot.getSysIdConstant("FEED_FORWARD_LINEAR_V").asDouble(),
            Robot.getSysIdConstant("FEED_FORWARD_LINEAR_A").asDouble(),
            Robot.getSysIdConstant("FEED_FORWARD_ANGULAR_V").asDouble(),
            Robot.getSysIdConstant("FEED_FORWARD_ANGULAR_A").asDouble());
  }
}
