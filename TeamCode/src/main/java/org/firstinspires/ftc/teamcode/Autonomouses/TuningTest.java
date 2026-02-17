package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Crawler.RobotOrient.ROMovementEngine;
import org.firstinspires.ftc.teamcode.Crawler.RobotOrient.Team;

@Autonomous
public class TuningTest extends ROMovementEngine {
    @Override
    public void runPath() {
// Generated Path (robot-oriented)
// Start: x=-1.41 y=-1.42 h=45deg

        turnPID(45); // face start heading
        turnPID(45);
        drivePID(1.76, 45);
        turnPID(38); // waypoint heading
        arc(0.97, 0.80, t -> t
                .at(0.0, 38)
                .at(0.20, -79)
                .at(0.40, -92)
                .at(0.60, -105)
                .at(0.80, -118)
                .at(1.0, -85)
        );
        turnPID(-80);
        drivePID(0.17, -80);
        turnPID(-92); // waypoint heading
        turnPID(-92);
        drivePID(0.15, -92);
        turnPID(-97); // waypoint heading
    }
}
