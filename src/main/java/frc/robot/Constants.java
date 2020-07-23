/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {
    public interface Drivetrain {
        public static final int LEFT_TOP_MOTOR_PORT = 1; 
        public static final int LEFT_MID_MOTOR_PORT = 2; 
        public static final int LEFT_BOTTOM_MOTOR_PORT = 3; 

        public static final int RIGHT_TOP_MOTOR_PORT = 4; 
        public static final int RIGHT_MID_MOTOR_PORT = 5; 
        public static final int RIGHT_BOTTOM_MOTOR_PORT = 6; 

        public static final double trackWidth = 2; 

        public interface FF {
            // what are the gains for feedforward?
            public static final double ks = 1; 
            public static final double kv = 1; 
            public static final double ka = 1; 
        }

        public interface PID {
            // what are the gains for PID? 
            public static final double kp = 1; 
            public static final double ki = 0; 
            public static final double kd = 0; 
        }
    }
}
