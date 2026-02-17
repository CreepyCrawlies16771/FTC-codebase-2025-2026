package org.firstinspires.ftc.teamcode.Crawler;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Crawler.RobotOrient.IndexerRotation;

public class Robot {

    // --- 1. CONSTANTS ---
    static final double COUNTS_PER_MOTOR_REV = 288.0; // REV Core Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;

    // Servo calibration constants to prevent "pushing the green"
    // Values must be between 0.0 and 1.0
    static final double LIFTER_FIRE_POS = 0.1; // Adjusted to not go too high/low
    static final double LIFTER_HOME_POS = 0.85; // Fully retracted

    DcMotor frontRight, frontLeft, backRight, backLeft;
    DcMotor shooterLeft, shooterRight;
    public DcMotor indexer;
    DcMotor gobbler;

    public MotorEx leftEncoder, rightEncoder, centerEncoder;

    public ColorSensor ballColorSensor;
    public Servo lifter;
    public IMU imu;

    int counter = 0;
    int indexerHome = 0;

    // Tracks the absolute mathematical target so mechanical slop doesn't accumulate
    private int targetPositionTicks = 0;
    private static int alphaThreshold = 0;

    public Robot(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft = hwMap.get(DcMotor.class, "leftShoot");
        shooterRight = hwMap.get(DcMotor.class, "rightShoot");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- INDEXER SETUP ---
        indexer = hwMap.get(DcMotor.class, "indexer");
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder to 0 when robot starts
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPositionTicks = 0;

        gobbler = hwMap.get(DcMotor.class, "gobbler");
        gobbler.setDirection(DcMotorSimple.Direction.FORWARD);

        ballColorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        lifter = hwMap.get(Servo.class, "lifter");
        // Ensure lifter starts in the home position
        lifter.setPosition(LIFTER_HOME_POS);

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    }

    public void activateShooters(boolean stop) {
        if (stop) {
            shooterRight.setPower(0);
            shooterLeft.setPower(0);
        } else {
            shooterLeft.setPower(RobotConfig.RobotBase.maxShooterSpeed);
            shooterRight.setPower(RobotConfig.RobotBase.maxShooterSpeed);
        }
    }

    public void powerDriveTrain(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(-frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(-backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void activateGobbler(boolean gooble, boolean revese) {
        if (gooble) {
            if (revese) gobbler.setPower(-1); else gobbler.setPower(1);
        } else {
            gobbler.setPower(0);
        }
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        powerDriveTrain(
                frontLeftPower / maxPower,
                frontRightPower / maxPower,
                backLeftPower / maxPower,
                backRightPower / maxPower
        );
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        this.drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    public void rotateIndexer(double degrees, IndexerRotation direction) {
        int tickMovement = (int) (degrees * COUNTS_PER_DEGREE);

        if (direction == IndexerRotation.COUNTERCLOCKWISE) {
            targetPositionTicks += tickMovement;
        } else {
            targetPositionTicks -= tickMovement;
        }

        indexer.setTargetPosition(targetPositionTicks);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(0.4); // Lower power to reduce bouncing/slop

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double timeoutSeconds = 3.0;

        while (indexer.isBusy() && timer.seconds() < timeoutSeconds) {
            // Waiting for motor
        }

        // --- JAM RECOVERY ---
        if (timer.seconds() >= timeoutSeconds) {
            gobbler.setPower(1.0);
            indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            indexer.setPower(0.5); // Reverse briefly

            ElapsedTime recoveryTimer = new ElapsedTime();
            while (recoveryTimer.seconds() < 0.5);

            gobbler.setPower(0);
            indexer.setTargetPosition(targetPositionTicks);
            indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexer.setPower(0.4);

            ElapsedTime finalTimer = new ElapsedTime();
            while (indexer.isBusy() && finalTimer.seconds() < 1.0);
        }

        indexer.setPower(0);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shootSequence() throws InterruptedException {
        double indexAngle = 120;
        activateShooters(false);

        // Move from Intake (0°) to first Shooter alignment (60°)
        rotateIndexer(60, IndexerRotation.CLOCKWISE);
        sleep(800);

        for (int i = 0; i < 3; i++) {
            sleep(500);

            // Fire sequence
            lifter.setPosition(LIFTER_FIRE_POS);
            sleep(1000); // Wait for servo to move
            lifter.setPosition(LIFTER_HOME_POS);
            sleep(500);

            // Move to next slot unless it's the last shot
            if (i < 2) {
                rotateIndexer(indexAngle, IndexerRotation.CLOCKWISE);
                sleep(400);
            }
        }

        activateShooters(true);

        // Move the final 60° back to the next Intake position (360° total rotation)
        rotateIndexer(60, IndexerRotation.CLOCKWISE);
    }

    public void cycleIndexer() throws InterruptedException {
        rotateIndexer(120, IndexerRotation.CLOCKWISE);
    }

    public void realignIndexer() {
        targetPositionTicks = indexerHome;
        indexer.setTargetPosition(targetPositionTicks);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(0.5);

        ElapsedTime timer = new ElapsedTime();
        while (indexer.isBusy() && timer.seconds() < 5.0);

        indexer.setPower(0);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBallThere() {
        return ballColorSensor.alpha() <= alphaThreshold;
    }
}