package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.PID;

@TeleOp(name = "MainTele")
public class Main extends OpMode {
    private Robot robot;
    private double START_ANGLE;
    private boolean pressed = false;
    private PID pid = new PID(0.025, 0, 0);

    @Override
    public void init() {
        robot = new Robot(this);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Ready!");
        telemetry.update();

        robot.marker.setPosition(1);
    }

    @Override
    public void loop() {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double left = -(gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y));
        double right = -(gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y));

        if (gamepad1.right_bumper) {

            if (!pressed) {
                //Runs once
                START_ANGLE = robot.gyro.getContinuosAngle();
                pressed = true;
            }

            double error = robot.gyro.getContinuosAngle() - START_ANGLE;
            double power = pid.getOutput(error);

            robot.lf.setPower(.75 + power);
            robot.rf.setPower(-.75 - power);
            robot.lb.setPower(-.75 + power);
            robot.rb.setPower(.75 - power);

            telemetry.addData("error", error);
            telemetry.addData("correction", power);
            telemetry.addData("current angle", robot.gyro.getContinuosAngle());
            telemetry.addData("target angle", START_ANGLE);

        } else if (gamepad1.left_bumper) {

            if(!pressed) {
                //Runs once
                START_ANGLE = robot.gyro.getContinuosAngle();
                pressed = true;
            }

            double error = robot.gyro.getContinuosAngle() - START_ANGLE;
            double power = pid.getOutput(error);

            robot.lf.setPower(-.75 + power);
            robot.rf.setPower(.75 - power);
            robot.lb.setPower(.75 + power);
            robot.rb.setPower(-.75 - power);

            telemetry.addData("error", error);
            telemetry.addData("correction", power);
            telemetry.addData("current angle", robot.gyro.getContinuosAngle());
            telemetry.addData("target angle", START_ANGLE);

        } else if (gamepad1.dpad_up) {
            robot.lf.setPower(.25);
            robot.rf.setPower(.25);
            robot.lb.setPower(.25);
            robot.rb.setPower(.25);
        } else if (gamepad1.dpad_down) {
            robot.lf.setPower(-.25);
            robot.rf.setPower(-.25);
            robot.lb.setPower(-.25);
            robot.rb.setPower(-.25);
        } else {
            robot.rf.setPower(right);
            robot.lb.setPower(left);
            robot.rb.setPower(right);
            robot.lf.setPower(left);
            pressed = false;
        }

        if (gamepad2.right_bumper) {
            robot.sweeper.setPower(1);
        } else if (gamepad2.left_bumper) {
            robot.sweeper.setPower(-1);
        } else {
            robot.sweeper.setPower(0);
        }

        if (gamepad2.dpad_up && !robot.hookUp.isPressed()) {
            robot.winch.setPower(0.75);
        } else if (gamepad2.dpad_down && !robot.hookDown.isPressed()) {
            robot.winch.setPower(-.75);
        } else {
            robot.winch.setPower(0);
        }

        if (gamepad2.y && !robot.slideLimit.isPressed()) {
            robot.lift.setPower(1);
        } else if (gamepad2.a) {
            robot.lift.setPower(-0.75);
        } else {
            robot.lift.setPower(0);
        }

        if (gamepad2.b) {
            robot.flipper.setPosition(1);
        } else {
            robot.flipper.setPosition(0.1);
        }

        telemetry.addData("pressed", pressed);
        telemetry.addData("lf", robot.lf.getPower());
        telemetry.addData("lb", robot.lb.getPower());
        telemetry.addData("rf", robot.rf.getPower());
        telemetry.addData("rb", robot.rb.getPower());
        telemetry.addData("LIMIT SWITCH", robot.slideLimit.isPressed());
        telemetry.update();

    }

    @Override
    public void stop() {
        robot.gyro.stop();
    }
}
