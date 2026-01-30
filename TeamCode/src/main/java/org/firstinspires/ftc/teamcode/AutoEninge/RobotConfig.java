package org.firstinspires.ftc.teamcode.AutoEninge;

/**
 * Internal configuration class, primarily to allow for simpler configuration
 */
@RobotConfig
public static class RobotConfig {
    public static final double ODO_WHEEL_DIAMETER_METERS = 0.048;
    public static final double ENCODER_TICKS_PER_REV = 2000;
    public static final double ODO_WHEEL_CIRCUMFERENCE = ODO_WHEEL_DIAMETER_METERS * Math.PI;
    public static final double TICKS_PER_METER = (ENCODER_TICKS_PER_REV / ODO_WHEEL_CIRCUMFERENCE);

    public static double Kp = 0.6;
    public static double Kd = 0;
    public static double Ki = 0;

    // FIXED: Set strafe coefficients to non-zero values
    public static double strafe_Kp = 1.85;
    public static double strafe_Ki = 0.00015;
    public static double strafe_Kd = 0;

    public static final double STEER_P = 0.02;
    public static final double MIN_POWER = 0.1;

    public static double timeoutSecs = 4;
}