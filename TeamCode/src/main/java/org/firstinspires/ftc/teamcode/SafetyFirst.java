package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Safety First")
public class SafetyFirst extends LinearOpMode {
    RobotHardware robot;

    private void waitFor(long ms){
        try {
            Thread.sleep(ms);
        }catch(Exception e){}
        robot.chassis(0,0,0);
    }

    @Override
    public void runOpMode(){
        robot = new RobotHardware();
        robot.init(hardwareMap, "");
        waitForStart();
        robot.chassis(0,-0.3,0);
        waitFor(500);
        robot.chassis(0.3,0,0);
        waitFor(500);
        robot.chassis(0,0.3,0);
        waitFor(500);
    }
}
