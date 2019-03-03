package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.InvictaCVHull;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.invictarobotics.invictavision.CameraViewDisplay;

@Autonomous(name = "CraterSide")
public class CraterDropBlock  extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this);
        robot.marker.setPosition(0);

        InvictaCVHull invictaCV = new InvictaCVHull();
        invictaCV.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        invictaCV.enable();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        double START_ANGLE = robot.gyro.getContinuosAngle();

        while(!robot.hookUp.isPressed() && opModeIsActive() && !isStopRequested()) {
            robot.winch.setPower(1);
        }

        robot.winch.setPower(0);

        robot.drive.move2(-6);

        robot.drive.strafe('l', .5);

        robot.drive.turn(-115);

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        PID pid = new PID(0.0014,0.00267,0.00049);
        pid.setLimits(-1, 1);
        double error;

        while(opModeIsActive() && !isStopRequested()) {
            if(invictaCV.found) {
                error = invictaCV.error - 100;
                if(Math.abs(error) < 4) {
                    break;
                }
                double power = pid.getOutput(error);

                robot.lb.setPower(-power);
                robot.rb.setPower(power);

                telemetry.addData("error", error);
                telemetry.addData("power", power);
                telemetry.addData("area", invictaCV.area);
                telemetry.update();
            } else {
                robot.drive.angularVelocity(45);
            }
        }

        robot.lb.setPower(0);
        robot.rb.setPower(0);
        robot.rf.setPower(0);
        robot.lf.setPower(0);

        double NEW_ANGLE = robot.gyro.getContinuosAngle() - START_ANGLE;

        telemetry.addData("angle", NEW_ANGLE);
        telemetry.update();

        if(NEW_ANGLE < -100) {
            robot.drive.move2(-30.5);
            sleep(100);
            robot.drive.move2(13);
            robot.drive.turn(-(robot.gyro.getContinuosAngle() - START_ANGLE));
            robot.drive.move2(-65);

        } else if(NEW_ANGLE > -80) {
            robot.drive.move2(-31.5);
            sleep(100);
            robot.drive.move2(23.75);
            robot.drive.turn(-(robot.gyro.getContinuosAngle() - START_ANGLE ));
            robot.drive.move2(-45);

        } else {
            robot.drive.move2(-30.125);
            sleep(100);
            robot.drive.move2(15);
            robot.drive.turn(-(robot.gyro.getContinuosAngle() - START_ANGLE ));
            robot.drive.move2(-40.5);

        }

        invictaCV.disable();

        robot.drive.turn(-(robot.gyro.getContinuosAngle() - START_ANGLE)+45);

        robot.drive.move2(-50);

        robot.marker.setPosition(1);

        robot.gyro.stop();

    }

    private double squarePower(double time) {
        return (Math.cos(time)+Math.cos(3*(time))+Math.cos(5*(time)))/3;
    }
}
