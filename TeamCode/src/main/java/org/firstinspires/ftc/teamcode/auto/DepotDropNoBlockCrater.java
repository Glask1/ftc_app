package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled

//@Autonomous(name = "DepoDropNoBlockCrater")

public class DepotDropNoBlockCrater extends LinearOpMode {

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

        robot.drive.move(-70, -.5);

        robot.marker.setPosition(0);

        sleep(1000);

        robot.drive.turn(45);

        sleep(100);

        robot.drive.strafe('l', .25);

        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);

        sleep(500);

        robot.drive.move(110, .5);

        requestOpModeStop();

    }

    private double squarePower(double time) {
        return (Math.cos(time)+Math.cos(3*(time))+Math.cos(5*(time)))/3;
    }

    @Override
    public double getRuntime() {
        return super.getRuntime();
    }
}
