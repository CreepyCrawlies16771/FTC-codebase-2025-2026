package org.firstinspires.ftc.teamcode.AutoEninge;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class AutoEngine extends LinearOpMode {

    // --- TUNING VARIABLES (Adjust these for your robot) ---
    // Calculate: (Ticks per rotation of encoder) / (Circumference of deadwheel)
    // --- ODOMETRY HARDWARE CONSTANTS ---
// goBILDA 48mm Pod: https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/
    protected final double ODO_WHEEL_DIAMETER_METERS = 0.048;
    protected final double ENCODER_TICKS_PER_REV = 8192; // PPR for SRE Magnetic Encoder

    // --- AUTO-CALCULATED CONSTANTS ---
    protected final double ODO_WHEEL_CIRCUMFERENCE = ODO_WHEEL_DIAMETER_METERS * Math.PI;
    protected final double TICKS_PER_METER = ENCODER_TICKS_PER_REV / ODO_WHEEL_CIRCUMFERENCE;


    // PID Coefficients
    protected double Kp = 0.8;//1.2  // Power: how fast it moves toward target
    protected double Kd = 0.3;//0.05; // Dampening: prevents shaking/overshoot
    protected double Ki = 0.01;//0.01; // Integral: handles friction at the very end

    protected final double STEER_P = 0.03; // How aggressively it fixes its angle
    protected final double MIN_POWER = 0.1; // Minimum power to overcome friction

    // --- HARDWARE ---
    protected DcMotor backLeft, backRight, frontLeft, frontRight;
    protected DcMotor leftOdo, rightOdo, centerOdo; // Deadwheels
    protected IMU imu;

    public abstract void runPath();

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Drivetrain Hardware
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        leftOdo   = hardwareMap.get(DcMotor.class, "frontLeft");
        rightOdo  = hardwareMap.get(DcMotor.class, "backRight");
        centerOdo = hardwareMap.get(DcMotor.class, "backLeft");

        setMotorBehavior();

        // 3. IMU Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        resetOdometry();

        telemetry.addData("Status", "Odometry Engine Ready");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            runPath();
        }
    }

    // --- PID MOVEMENT COMMANDS ---

    /**
     * Move straight (Forward/Backward) using PID and Deadwheels
     */
    public void drivePID(double targetMeters, int targetAngle) {
        double targetTicks = targetMeters * TICKS_PER_METER;
        double error = targetTicks;
        double lastError = 0;
        double integral = 0;

        resetOdometry();

        while (opModeIsActive() && Math.abs(error) > 50) {
            double currentPos = (leftOdo.getCurrentPosition() + rightOdo.getCurrentPosition()) / 2.0;
            error = targetTicks - currentPos;

            // PID Logic
            double derivative = error - lastError;
            integral += error;

            // Anti-windup cap
            if (Math.abs(error) < (0.1 * TICKS_PER_METER)) { // Only use Integral when close
                integral = Math.max(-20, Math.min(20, integral));
            } else {
                integral = 0;
            }

            double power = (Kp * (error / TICKS_PER_METER)) + (Ki * integral) + (Kd * derivative);

            // Steering with Angle Wrap
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(currentYaw - targetAngle) * -STEER_P;

            power = Math.max(-0.7, Math.min(0.7, power));
            if (Math.abs(power) < MIN_POWER) power = Math.signum(power) * MIN_POWER;

            applyDrivePower(power, steer);
            lastError = error;
        }
        stopRobot();
    }

    /**
     * Strafe (Left/Right) using Center Deadwheel
     */
    public void strafePID(double targetMeters, int targetAngle) {
        double targetTicks = targetMeters * TICKS_PER_METER;
        double error = targetTicks;
        resetOdometry();

        while (opModeIsActive() && Math.abs(error) > 50) {
            double currentPos = centerOdo.getCurrentPosition();
            error = targetTicks - currentPos;

            double power = (Kp * (error / TICKS_PER_METER)); // Simplified P-loop for strafe
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(currentYaw - targetAngle) * -STEER_P;

            power = Math.max(-0.7, Math.min(0.7, power));

            // Strafe power mapping
            backLeft.setPower(-power + steer);
            backRight.setPower(-power + steer);
            frontLeft.setPower(power + steer);
            frontRight.setPower(-power - steer);
        }
        stopRobot();
    }

    // --- HELPERS ---

    private void applyDrivePower(double p, double s) {
        backLeft.setPower(p - s);
        backRight.setPower(p + s);  // Changed from -p to p
        frontLeft.setPower(p - s);  // Ensure this matches your wiring
        frontRight.setPower(p + s);
    }
    private void stopRobot() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(150); // Small pause for stability
    }

    private void resetOdometry() {
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setMotorBehavior() {
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
}