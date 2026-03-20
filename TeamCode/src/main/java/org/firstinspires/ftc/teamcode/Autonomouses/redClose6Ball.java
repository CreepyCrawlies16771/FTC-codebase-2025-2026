package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Crawler.RobotOrient.ROMovementEngine;

@Autonomous
public class redClose6Ball extends ROMovementEngine{
    @Override
    public void runPath() throws InterruptedException {
        // Robot-Oriented Path (using OdoEngine)
// Start: x=0.00 y=0.00 h=0deg (always relative to origin)

        turnPID(0); // face start heading
        turnPID(-47);
        drivePID(1.62, -47);
        turnPID(-31); // waypoint heading
        turnPID(91);
        drivePID(0.84, 91);
        turnPID(93); // waypoint heading
        turnPID(92);
        drivePID(0.16, 92);
        turnPID(86); // waypoint heading
        turnPID(90);
        drivePID(0.15, 90);
        arc(1.21, 0.60, t -> t
                .at(0.0, 0)
                .at(0.07, -61)
                .at(0.13, -64)
                .at(0.20, -68)
                .at(0.27, -73)
                .at(0.33, -77)
                .at(0.40, -81)
                .at(0.47, -86)
                .at(0.53, -90)
                .at(0.60, -95)
                .at(0.67, -99)
                .at(0.73, -104)
                .at(0.80, -108)
                .at(0.87, -112)
                .at(0.93, -116)
                .at(1.0, -46)
        );
    }
}
