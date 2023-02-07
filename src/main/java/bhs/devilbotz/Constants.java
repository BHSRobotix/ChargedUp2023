// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bhs.devilbotz;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

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
    /**
     * The deadband for the joystick. This is the minimum value that the joystick must be at to move
     * the robot.
     */
    public static final double JOYSTICK_DEADBAND = 0.035;

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
