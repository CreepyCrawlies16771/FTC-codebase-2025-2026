package org.firstinspires.ftc.teamcode.Crawler.Navigation;
import org.firstinspires.ftc.teamcode.Crawler.Core.PIDController;
import org.firstinspires.ftc.teamcode.Crawler.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;

public class Follower {
    private final Robot robot;
    // Scale translational gains for mm (original gains were for inches)
    // 1 inch = 25.4 mm, so divide translational gains by ~25.4
    public PIDController xPID = new PIDController(0.07 / 25.4, 0, 0.005 / 25.4, 0.02 / 25.4);
    public PIDController yPID = new PIDController(0.07 / 25.4, 0, 0.005 / 25.4, 0.02 / 25.4);
    // Heading controller operates on degrees and uses angular PID
    public PIDController hPID = new PIDController(0.05, 0, 0.001, 0.01);

    public Follower(Robot robot) { this.robot = robot; }

    public boolean update(double tx, double ty, double th, double maxSpeed, Localizer loc) {
        double xErr = tx - loc.x;
        double yErr = ty - loc.y;
        double hErr = th - loc.getHeadingDegrees();

        // Field Centric Transformation
        // Convert heading (degrees) -> radians for trig
        double rads = Math.toRadians(-loc.getHeading());
        double rotX = xErr * Math.cos(rads) - yErr * Math.sin(rads);
        double rotY = xErr * Math.sin(rads) + yErr * Math.cos(rads);

        // Calculate raw PID outputs
        double drive = xPID.calculate(rotX, 0);
        double strafe = yPID.calculate(rotY, 0);
        // Use angular PID (with wrapping) for heading
        double turn = hPID.calculateAngle(hErr, 0);

        // Normalize and apply Velocity Limit (maxSpeed)
        double combined = Math.abs(drive) + Math.abs(strafe) + Math.abs(turn);
        double scale = Math.max(combined, 1.0 / maxSpeed);

        robot.powerDriveTrain(
                (drive + strafe + turn) / scale,
                (drive - strafe - turn) / scale,
                (drive - strafe + turn) / scale,
                (drive + strafe - turn) / scale
        );

        // Returns true only if we are settled (used for non-dynamic steps)
        // Relax thresholds to realistic mm/deg values
        return Math.abs(xErr) < 10.0 && Math.abs(yErr) < 10.0 && Math.abs(hErr) < 3.0;
    }
}