package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Crawler.RobotOrient.ROMovementEngine;

@Autonomous
public class blueClose6BallAuto extends ROMovementEngine {
    @Override
    public void runPath() throws InterruptedException {
        // Robot-Oriented Path (using OdoEngine)
        // Start: x=0.00 y=0.00 h=0deg (always relative to origin)
        turnPID(0); // face start heading
        turnPID(46);
        drivePID(1.48, 46);
        turnPID(-87);
        drivePID(0.70, -87);
        turnPID(-88); // waypoint heading
        turnPID(-90);
        drivePID(0.19, -90);
        turnPID(-93); // waypoint heading
        turnPID(-87);
        drivePID(0.12, -87);
        turnPID(-82); // waypoint heading
        arc(1.05, 0.60, t -> t
                .at(0.0, 0)
                .at(0.07, 120)
                .at(0.13, 117)
                .at(0.20, 113)
                .at(0.27, 108)
                .at(0.33, 104)
                .at(0.40, 100)
                .at(0.47, 95)
                .at(0.53, 91)
                .at(0.60, 86)
                .at(0.67, 82)
                .at(0.73, 77)
                .at(0.80, 73)
                .at(0.87, 69)
                .at(0.93, 65)
                .at(1.0, 50)
        );
    }
}
