package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Crawler.RobotOrient.ROMovementEngine;

@Autonomous
public class JustGoBack extends ROMovementEngine {
    @Override
    public void runPath() throws InterruptedException {
        drivePID(0.5, 0);
    }
}
