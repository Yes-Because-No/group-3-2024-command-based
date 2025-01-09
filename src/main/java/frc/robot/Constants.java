package frc.robot;

/*
 *The organization in this Constants class is kind of off right now.
 *I just didn't change it because I was too lazy to. 
 *Remember to organize it by subsystems (this was written before I split arm into intake & arm) 
*/
public final class Constants {
    public static final class Drivetrain {
        // CAN IDS for Spark Maxes
        public static final class CANIDs {
            public static final int LEFT = 15;
            public static final int RIGHT = 14;
        }

        // Drivetrain gear ratio (189 in, 20 out)
        public static final double GEAR_RATIO = 189 / 20.0;
        public static final int CURRENT_LIMIT = 40;
    }

    public static final class Arm {
        // arm & intake Spark Maxes
        public static final class CANIDs {
            public static final int INTAKE = 12;
            public static final int ARM = 13;
        }

        // constants for feedforward - from sysid
        public static final class Feedforward {
            public static final double kS = 0.01141;
            public static final double kG = -0.022958;
            public static final double kV = 0.0018501;
        }

        // pid constants - also from sysid
        public static final class PID {
            public static final double kP = 2.6947;
            public static final double kI = 0;
            public static final double kD = 0.053863;

            // make pid smoother and controlled
            public static final class constraints {
                public static final int kMAXV = 5;
                public static final int kMAXACC = 3;
            }

            // all the pts where the arm will be set to
            public static final class setpoints {
                public static final double GROUND = -2.0;
                public static final double MIDDLE = -0.93;
                public static final double HIGH = 0;
            }
        }

        // safe arm current limit and intake current limit
        public static final int ARM_CURRENT_LIMIT = 5;
        public static final int INTAKE_CURRENT_LIMIT = 20;
        // arm gear ratio - needed for accurate PID control
        public static final double ARM_GEAR_RATIO = (3.0 / 100);
        public static final double ENCODER_TO_RADIANS = ARM_GEAR_RATIO * Math.PI * 2;
    }

    // drag controller to slot 0 if not already at it (in driver station)
    public static final int CONTROLLER_PORT = 0;
    // softens arcade drive input
    public static final double SLEW_RATE_LIMIT = 2;
}
