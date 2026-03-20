package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Crawler.RobotOrient.ROMovementEngine;

@Autonomous
public class wolfsparkauto extends ROMovementEngine {
    @Override
    public void runPath() throws InterruptedException {

        turnPID(0); // face start heading
        turnPID(-179);
        drivePID(-1, -179);
        turnPID(39); // waypoint heading
        strafePID(1.32, 1);
        turnPID(-96); // waypoint heading
    }
}
