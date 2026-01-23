package org.firstinspires.ftc.teamcode.AutoEninge;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Robot {

    DcMotor frontRight , frontLeft, backRight, backLeft;

    DcMotor shooterLeft, shooterRight;

    DcMotor indexer, gobbler;

    public ColorSensor ballColorSensor;

    Servo lifter;

    public IMU imu;

    int counter = 0;

    private static int alphaThreshold = 0;

    public Robot(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class , "frontLeft");
        frontRight = hwMap.get(DcMotor.class , "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");

        shooterLeft = hwMap.get(DcMotor.class, "leftShoot");
        shooterRight = hwMap.get(DcMotor.class, "rightShoot");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        indexer = hwMap.get(DcMotor.class, "indexer");
        gobbler = hwMap.get(DcMotor.class, "gobbler");
        gobbler.setDirection(DcMotorSimple.Direction.REVERSE);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ballColorSensor = hwMap.get(ColorSensor.class , "colorSensor");

        lifter = hwMap.get(Servo.class, "lifter");

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    }

    public void activateShooters(boolean stop) {

        if(stop) {shooterRight.setPower(0); shooterLeft.setPower(0);} else {
            shooterLeft.setPower(0.55);
            shooterRight.setPower(-0.55);
        }

    }

    public void powerDriveTrain(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(-frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(-backLeftPower);
        backRight.setPower(backRightPower);
    }
    public void activateGobbler(boolean gooble) { //Feature not a bug!
        if (gooble) {
            gobbler.setPower(-1);
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

    public void rotateIndexer(IndexerRotation direction) throws InterruptedException {

        if (direction == IndexerRotation.COUNTERCLOCKWISE){
            indexer.setPower(-0.25);
            sleep(750);
            indexer.setPower(0);
        }
        else {
            indexer.setPower(0.25);
            sleep(750);
            indexer.setPower(0);
        }
    }
    public void shootSequence() throws InterruptedException {

        //ODER: Mite links rechts
        activateShooters(false);
        sleep(1000);

        lifter.setPosition(-0.6);
        sleep(500);
        lifter.setPosition(1);
        sleep(500);

        rotateIndexer(IndexerRotation.CLOCKWISE);

        lifter.setPosition(-0.8);
        sleep(500);
        lifter.setPosition(1);
        sleep(500);

        rotateIndexer(IndexerRotation.CLOCKWISE);

        lifter.setPosition(-0.6);
        sleep(500);
        lifter.setPosition(1);
        sleep(500);


        activateShooters(true);



        activateShooters(true);


    }

    public void cycleIndexer() throws InterruptedException {
        rotateIndexer(IndexerRotation.COUNTERCLOCKWISE);
        sleep(1000);
    }

    public boolean isBallThere() {
        return ballColorSensor.alpha() <= alphaThreshold;
    }


}
