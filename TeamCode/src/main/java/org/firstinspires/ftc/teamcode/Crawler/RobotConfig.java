package org.firstinspires.ftc.teamcode.Crawler;

import com.acmerobotics.dashboard.config.Config;

public class RobotConfig {

    @Config
    public static class FieldOriented {
        public static String encodeLeftName = "frontLeft";
        public static String encoderRightName = "backRight";
        public static String encoderCenterName = "backLeft";

        public static double defaultLookAheadDistance = 5;
        public static double FINISH_THRESHOLD_CM = 2.0;

        public static final double STEER_P = 0.02;
        public static final double MIN_POWER = 0.1;
    }

    @Config
    public static class RobotOriented {
        public static double Kp = 0.6;
        public static double Kd = 0;
        public static double Ki = 0;

        public static double strafe_Kp = 1.85;
        public static double strafe_Ki = 0.00015;
        public static double strafe_Kd = 0;

        // REMOVED 'final' here
        public static double STEER_P = 0.02;
        public static final double MIN_POWER = 0.1;
    }

    @Config
    public static class RobotBase {
        public static double timeoutSecs = 4;

        // These will now show up!
        public static double maxShooterSpeed = 1;
        public static double maxGobblerSpeed = 1;

        public static double ODO_WHEEL_DIAMETER_METERS = 0.048;
        public static double ENCODER_TICKS_PER_REV = 2000;

        public static final double TRACK_WIDTH = 0;
        public static final double CENTER_WHEEL_OFFSET = 0;

        public static final double ODO_WHEEL_CIRCUMFERENCE = ODO_WHEEL_DIAMETER_METERS * Math.PI;
        public static final double TICKS_PER_METER = (ENCODER_TICKS_PER_REV / ODO_WHEEL_CIRCUMFERENCE);
        public static final double TICKS_PER_CM = TICKS_PER_METER * 100;
    }



}