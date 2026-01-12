package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Use ME! Driver")
public class driver extends OpMode{

        DcMotor frontRight;
        DcMotor backRight;
        DcMotor frontLeft;
        DcMotor backLeft;
        DcMotor leftShooter;
        DcMotor rightShooter;

        DcMotor gobbler;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;


        boolean shootOn = false;
        boolean gobbleOn = false;

        @Override
        public void init() {
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");

            leftShooter = hardwareMap.get(DcMotor.class, "leftShoot");
            rightShooter = hardwareMap.get(DcMotor.class, "rightShoot");

            gobbler = hardwareMap.get(DcMotor.class, "gobbler");

            leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
            rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

            gobbler.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public void loop() {
            double leftY = gamepad1.left_stick_y;
            double rightY = gamepad1.right_stick_y;
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            frontLeftPower = leftY + leftX;
            frontRightPower = rightY - rightX;
            backLeftPower = leftY - leftX;
            backRightPower = rightY + rightX;

            powerDriveTrain(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            shootOn = gamepad2.right_bumper && !shootOn;
            activateShooter(shootOn);

            gobbleOn = gamepad2.left_bumper && !gobbleOn;
            activateGobbler(gobbleOn);

        }

        void powerDriveTrain(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

        void activateShooter(boolean shoot) {
            if (shoot) {
                leftShooter.setPower(1);
                rightShooter.setPower(1);
            } else {
                leftShooter.setPower(0);
                rightShooter.setPower(0);
            }
        }

        void activateGobbler(boolean gooble) {
            if (gooble) {
                gobbler.setPower(1);
            } else {
                gobbler.setPower(0);
            }
        }
    }

