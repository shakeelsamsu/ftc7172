package org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.gfdebug.ComputerDebugging;
import org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.gfdebug.Robot;

public class Main
{
    public static void main(final String[] args) {
        new Main().run();
    }

    public void run() {
        final ComputerDebugging computerDebugging = new ComputerDebugging();
        final Robot robot = new Robot();
        final OpMode opMode = (OpMode)new SSPursuitOpMode();
//        opMode.init();
        ComputerDebugging.clearLogPoints();
        final long startTime = System.currentTimeMillis();
        try {
            Thread.sleep(1000L);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        while (true) {
            opMode.loop();
            try {
                Thread.sleep(30L);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.update();
            ComputerDebugging.sendRobotLocation(robot);
            ComputerDebugging.sendLogPoint(new com.company.FloatPoint(Robot.worldXPosition, Robot.worldYPosition));
            ComputerDebugging.markEndOfUpdate();
        }
    }
}