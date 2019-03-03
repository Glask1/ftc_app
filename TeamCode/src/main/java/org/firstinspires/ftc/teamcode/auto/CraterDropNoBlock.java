package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.PID;

@Disabled

//@Autonomous(name = "CraterDropNoBlock")
public class CraterDropNoBlock extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this);
        robot.marker.setPosition(1);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while(!robot.hookUp.isPressed() && opModeIsActive()) {
            robot.winch.setPower(1);
        }

        robot.winch.setPower(0);

        robot.drive.move(-6, -.5);

        robot.drive.strafe('l', .5);

        robot.drive.turn(-90);

        robot.drive.move(-60, -.75);

        double start = getRuntime();
        while(getRuntime() - start < 3) {
            robot.lf.setPower(-1);
            robot.rf.setPower(-1);
        }
        robot.lf.setPower(0);
        robot.rf.setPower(0);

        requestOpModeStop();

    }

    private double squarePower(double time) {
        return (Math.cos(time)+Math.cos(3*(time))+Math.cos(5*(time)))/3;
    }
}
