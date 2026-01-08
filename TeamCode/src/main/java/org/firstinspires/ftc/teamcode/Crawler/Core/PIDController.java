package org.firstinspires.ftc.teamcode.Crawler.Core;


//import com.acmerobotics.dashboard.config.Config;
//
//@Config
public class PIDController {
    private final double kp, ki, kd, kf;
    private double lastError = 0, integral = 0;
    // Integral clamp to avoid wind-up
    private final double integralLimit = 1e6;

    public PIDController(double kp, double ki, double kd, double kf) {
        this.kp = kp; this.ki = ki; this.kd = kd; this.kf = kf;
    }

    /**
     * Linear PID calculation (no angle wrapping).
     * Use this for X/Y controllers where error is in linear units.
     */
    public double calculate(double target, double current) {
        double error = target - current;

        integral += error;
        // Anti-windup: clamp integral
        if (integral > integralLimit) integral = integralLimit;
        if (integral < -integralLimit) integral = -integralLimit;

        double derivative = error - lastError;
        lastError = error;
        return (error * kp) + (integral * ki) + (derivative * kd) + (Math.signum(error) * kf);
    }

    /**
     * Angular PID calculation (degrees). This performs angle wrapping
     * so the controller takes the shortest rotational path.
     */
    public double calculateAngle(double targetDeg, double currentDeg) {
        double error = targetDeg - currentDeg;
        // Angle Wrapping: ensures the robot takes the shortest turn (degrees)
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        integral += error;
        if (integral > integralLimit) integral = integralLimit;
        if (integral < -integralLimit) integral = -integralLimit;

        double derivative = error - lastError;
        lastError = error;
        return (error * kp) + (integral * ki) + (derivative * kd) + (Math.signum(error) * kf);
    }

    public void reset() {
        lastError = 0;
        integral = 0;
    }
}