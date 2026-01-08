package org.firstinspires.ftc.teamcode.Crawler.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Crawler.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;

@TeleOp(name = "Crawler Universal Tuner", group = "Crawler")
public class UniversalTuningOpMode extends LinearOpMode {
	private Localizer localizer;
	private Robot robot = new Robot();

	@Override
	public void runOpMode() throws InterruptedException {
		robot.init(hardwareMap);
		localizer = new Localizer(hardwareMap);
		localizer.resetPose(0, 0, 0);

		waitForStart();

		while (opModeIsActive()) {
			localizer.update();

			// --- MANUAL DRIVE (Default) ---
			double drive = -gamepad1.left_stick_y;
			double strafe = gamepad1.left_stick_x;
			double rotate = gamepad1.right_stick_x;

			// Standard 4-motor mix
			robot.powerDriveTrain(drive - rotate, drive + rotate, drive + rotate, drive - rotate);

			// --- TELEMETRY ---
			telemetry.addLine("=== CRAWLER TUNING HUB ===");
			telemetry.addData("X (mm)", "%.2f", localizer.getPoseX());
			telemetry.addData("Y (mm)", "%.2f", localizer.getPoseY());
			telemetry.addData("Heading (Deg)", "%.2f", localizer.getHeading());

			telemetry.addLine("\n--- ENCODER RAW ---");
			telemetry.addData("L", localizer.getLeftEncoder());
			telemetry.addData("R", localizer.getRightEncoder());
			telemetry.addData("C", localizer.getStrafeEncoder());

			telemetry.update();
		}
	}
}

