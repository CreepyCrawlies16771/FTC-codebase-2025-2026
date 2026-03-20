package org.firstinspires.ftc.teamcode.Crawler;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    // --- CONSTANTS FOR REV CORE HEX MOTOR ---
    static final double COUNTS_PER_MOTOR_REV = 288.0; // REV Core Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;

    // --- SERVO FIXES (MUST BE BETWEEN 0.0 AND 1.0) ---
    public static final double LIFTER_FIRE_POS = 0.25;
    public static final double LIFTER_HOME_POS = 0.85;

    // --- NON-BLOCKING STATE MACHINE ENUMS ---
public enum IndexerState { IDLE, ROTATING, FIRING_UP, FIRING_DOWN }
    public IndexerState indexerState = IndexerState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private int shootCount = 0;
    private boolean isShootSequence = false;

    DcMotor frontRight , frontLeft, backRight, backLeft;
    DcMotor shooterLeft, shooterRight;
    public DcMotor indexer;
    DcMotor gobbler;

    public MotorEx leftEncoder, rightEncoder, centerEncoder;

    public ColorSensor ballColorSensor;
    public Servo lifter;
    public IMU imu;

    int counter = 0;
     public int indexerHome = 0;

    // Absolute position tracker to eliminate slop
    public int targetPositionTicks = 0;
    private static int alphaThreshold = 0;

    public Robot(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class , "frontLeft");
        frontRight = hwMap.get(DcMotor.class , "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftEncoder = (MotorEx) hwMap.get(DcMotor.class, RobotConfig.encodeLeftName);
//        rightEncoder = (MotorEx) hwMap.get(DcMotor.class, RobotConfig.encoderRightName);
//        centerEncoder = (MotorEx) hwMap.get(DcMotor.class, RobotConfig.encoderCenterName);
//
//        leftEncoder.setDistancePerPulse(RobotConfig.TICKS_PER_CM);
//        rightEncoder.setDistancePerPulse(RobotConfig.TICKS_PER_CM);
//        centerEncoder.setDistancePerPulse(RobotConfig.TICKS_PER_CM);

        shooterLeft = hwMap.get(DcMotor.class, "leftShoot");
        shooterRight = hwMap.get(DcMotor.class, "rightShoot");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 2. UPDATED INDEXER SETUP ---
        indexer = hwMap.get(DcMotor.class, "indexer");
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Important: Reset encoder to 0 when robot starts
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPositionTicks = 0;

        gobbler = hwMap.get(DcMotor.class, "gobbler");
        gobbler.setDirection(DcMotorSimple.Direction.REVERSE);

        ballColorSensor = hwMap.get(ColorSensor.class , "colorSensor");

        lifter = hwMap.get(Servo.class, "lifter");
        lifter.setPosition(LIFTER_HOME_POS);

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    }

    // =========================================================================
    // --- EXACT ORIGINAL CODE (UNTOUCHED) ---
    // =========================================================================

    public void activateShooters(boolean stop) {
        if(stop) {
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

    public void activateGobbler(boolean gooble, boolean reverse) {
        if (gooble) {
            if(reverse) gobbler.setPower(RobotConfig.RobotBase.maxGobblerSpeed); else gobbler.setPower(-RobotConfig.RobotBase.maxGobblerSpeed);
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
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        powerDriveTrain(
                maxSpeed * (frontLeftPower / maxPower),
                maxSpeed * (frontRightPower / maxPower),
                maxSpeed * (backLeftPower / maxPower),
                maxSpeed * (backRightPower / maxPower)
        );
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    public boolean isBallThere() {
        return ballColorSensor.alpha() <= alphaThreshold;
    }

    // =========================================================================
    // --- NEW NON-BLOCKING INDEXER METHODS ---
    // =========================================================================

    /**
     * CALL THIS EVERY LOOP IN TELEOP
     */
    public void updateIndexer() {
        switch (indexerState) {
            case ROTATING:
                if (!indexer.isBusy()) {
                    indexer.setPower(0);
                    if (isShootSequence) {
                        if (shootCount < 3) {
                            stateTimer.reset();
                            indexerState = IndexerState.FIRING_UP;
                        } else {
                            activateShooters(true); // Stop shooters when done
                            isShootSequence = false;
                            indexerState = IndexerState.IDLE;
                        }
                    } else {
                        indexerState = IndexerState.IDLE;
                    }
                } else if (stateTimer.seconds() > 3.0) {
                    // Timeout/Jam Safe-Catch: Prevents infinite locking
                    indexer.setPower(0);
                    indexerState = IndexerState.IDLE;
                    isShootSequence = false;
                }
                break;

            case FIRING_UP:
                lifter.setPosition(LIFTER_FIRE_POS);
                if (stateTimer.milliseconds() > 800) {
                    stateTimer.reset();
                    indexerState = IndexerState.FIRING_DOWN;
                }
                break;

            case FIRING_DOWN:
                lifter.setPosition(LIFTER_HOME_POS);
                if (stateTimer.milliseconds() > 500) {
                    shootCount++;
                    activateShooters(false);
                    if (shootCount < 3) {
                        // Cycle to next ball
                        startRotation(120, IndexerRotation.CLOCKWISE);
                    } else {
                        // Move final 60 degrees back to Intake position
                        startRotation(60, IndexerRotation.CLOCKWISE);
                    }
                } else {activateShooters(true);}
                break;

            case IDLE:
            default:
                break;
        }
    }

    private void startRotation(double degrees, IndexerRotation direction) {
        int tickMovement = (int) (degrees * COUNTS_PER_DEGREE);

        if (direction == IndexerRotation.COUNTERCLOCKWISE){
            targetPositionTicks += tickMovement;
        } else {
            targetPositionTicks -= tickMovement;
        }

        indexer.setTargetPosition(targetPositionTicks);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(0.7);
        stateTimer.reset();
        indexerState = IndexerState.ROTATING;
    }

    public void rotateIndexer(double degrees, IndexerRotation direction) {
        if (indexerState != IndexerState.IDLE) return; // Ignore if busy
        isShootSequence = false;
        startRotation(degrees, direction);
    }

    public void shootSequence() {
        if (indexerState != IndexerState.IDLE) return; // Ignore if busy
        shootCount = 0;
        isShootSequence = true;
        activateShooters(false); // Start shooter motors
        startRotation(60, IndexerRotation.CLOCKWISE); // Align first ball
    }

    public void cycleIndexer() {
        if (indexerState != IndexerState.IDLE) return;
        isShootSequence = false;
        startRotation(120, IndexerRotation.CLOCKWISE);
    }

    public void  realignIndexer() {
        if (indexerState != IndexerState.IDLE) return;
        isShootSequence = false;
        targetPositionTicks = indexerHome;
        indexer.setTargetPosition(targetPositionTicks);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(0.7);
        stateTimer.reset();
        indexerState = IndexerState.ROTATING;
    }
}