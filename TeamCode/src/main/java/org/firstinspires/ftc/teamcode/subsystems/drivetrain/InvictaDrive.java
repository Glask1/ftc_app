package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.opencv.core.Mat;

public class InvictaDrive {

    private Robot robot;
    public PID pidMove = new PID(0,0,0);


    public InvictaDrive(Robot r) {
        this.robot = r;
    }

    public void turn(double targetAngle) {

        final double START_ANGLE = robot.gyro.getContinuosAngle();
        PID pidTurn = new PID(0.02, 0.0007, 0.00002);
        pidTurn.setLimits(-1, 1);
        final double ANGLE_THRESHOLD = 3;

        double currentAngle = robot.gyro.getContinuosAngle() - START_ANGLE;
        double error = targetAngle - currentAngle;

        while ((Math.abs(error) > ANGLE_THRESHOLD) && robot.LinearOpMode.opModeIsActive() && !robot.LinearOpMode.isStopRequested()) {

            double power = pidTurn.getOutput(error);

            if(power < 0.2 && power > 0.05) {
                power = 0.2;
            } else if(power > -0.2 && power < -0.5) {
                power = -0.2;
            }

            robot.lf.setPower(-power);
            robot.lb.setPower(-power);
            robot.rf.setPower(power);
            robot.rb.setPower(power);

            currentAngle = robot.gyro.getContinuosAngle() - START_ANGLE;
            error = targetAngle - currentAngle;

            robot.LinearOpMode.telemetry.addData("Z: ", robot.gyro.getContinuosAngle());
            robot.LinearOpMode.telemetry.update();
        }

        robot.lf.setPower(0);
        robot.lb.setPower(0);
        robot.rf.setPower(0);
        robot.rb.setPower(0);
    }

    public void move(double inches, double power) {

        final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double START_ANGLE = robot.gyro.getContinuosAngle();
        double currentAngle;
        pidMove.setValues(.1,0, 0);
        pidMove.setLimits(-1, 1);

        int newLeftTarget;
        int newRightTarget;
        double pivotCorrection;

        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.lb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightTarget = robot.rb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        robot.lb.setTargetPosition(newLeftTarget);
        robot.rb.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double error = (newLeftTarget - robot.lb.getCurrentPosition()) + (newRightTarget - robot.rb.getCurrentPosition());

        while ((robot.lb.isBusy()  || robot.rb.isBusy()) && Math.abs(error) > 800 && robot.LinearOpMode.opModeIsActive() && !robot.LinearOpMode.isStopRequested()) {
            error = (newLeftTarget - robot.lb.getCurrentPosition()) + (newRightTarget - robot.rb.getCurrentPosition());
            currentAngle = robot.gyro.getContinuosAngle() - START_ANGLE;
            pivotCorrection = pidMove.getOutput(currentAngle);
            robot.lb.setPower(power + pivotCorrection);
            robot.rb.setPower(power - pivotCorrection);
            robot.LinearOpMode.telemetry.addData("pivotCorrection", pivotCorrection);
            robot.LinearOpMode.telemetry.addData("left", robot.lb.getCurrentPosition());
            robot.LinearOpMode.telemetry.addData("right", robot.rb.getCurrentPosition());
            robot.LinearOpMode.telemetry.addData("difference", robot.lb.getCurrentPosition() - robot.rb.getCurrentPosition());
            robot.LinearOpMode.telemetry.update();
        }

        robot.lb.setPower(power);
        robot.rb.setPower(power);

        while ((robot.rb.isBusy() || robot.lb.isBusy()) && robot.LinearOpMode.opModeIsActive() && !robot.LinearOpMode.isStopRequested()) {

        }

        // Stop all motion;
        robot.rb.setPower(0);
        robot.lb.setPower(0);




        // Turn off RUN_TO_POSITION
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //turn(START_ANGLE - robot.gyro.getContinuosAngle());

    }
    public void move2(double inches) {

        final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        int newLeftTarget;
        int newRightTarget;
        int error;
        int threshold;
        int minError;
        double kp = 0.0004;
        PID pid = new PID(kp, 0, 0);
        pid.setLimits(-.7, .7);

        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.lb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightTarget = robot.rb.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        robot.lb.setTargetPosition(newLeftTarget);
        robot.rb.setTargetPosition(newRightTarget);

        error = (newLeftTarget - robot.lb.getCurrentPosition()) + (newRightTarget - robot.rb.getCurrentPosition());
        threshold = (int) (12 * COUNTS_PER_INCH);
        minError = 300;
        while (Math.abs(error) > threshold && robot.LinearOpMode.opModeIsActive() && !robot.LinearOpMode.isStopRequested()) {
            if(inches < 0) {
                robot.rb.setPower(-.75);
                robot.lb.setPower(-.75);
            } else if (inches > 0)  {
                robot.rb.setPower(.75);
                robot.lb.setPower(.75);
            }
            error = (newLeftTarget - robot.lb.getCurrentPosition()) + (newRightTarget - robot.rb.getCurrentPosition());
            robot.LinearOpMode.telemetry.addData("Error", error);
            robot.LinearOpMode.telemetry.update();
        }

        while (Math.abs(error) > minError && robot.LinearOpMode.opModeIsActive() && !robot.LinearOpMode.isStopRequested()) {
                robot.rb.setPower(pid.getOutput(error));
                robot.lb.setPower(pid.getOutput(error));
            error = (newLeftTarget - robot.lb.getCurrentPosition()) + (newRightTarget - robot.rb.getCurrentPosition());
            robot.LinearOpMode.telemetry.addData("Error", error);
            robot.LinearOpMode.telemetry.update();
        }

        // Stop all motion;
        robot.rb.setPower(0);
        robot.lb.setPower(0);

    }

    public void strafe(char dir, double time) {

        double START_ANGLE = robot.gyro.getContinuosAngle();
        PID pid = new PID(0.025, 0, 0);
        pid.setLimits(-1,1);
        ElapsedTime t = new ElapsedTime();
        t.startTime();

        while(robot.LinearOpMode.opModeIsActive() && t.seconds() < time && !robot.LinearOpMode.isStopRequested()) {
            if (dir == 'r') {

                double error = robot.gyro.getContinuosAngle() - START_ANGLE;
                double power = pid.getOutput(error);

                robot.lf.setPower(.75 + power);
                robot.rf.setPower(-.75 - power);
                robot.lb.setPower(-.75 + power);
                robot.rb.setPower(.75 - power);

            } else if (dir == 'l') {

                double error = robot.gyro.getContinuosAngle() - START_ANGLE;
                double power = pid.getOutput(error);

                robot.lf.setPower(-.75 + power);
                robot.rf.setPower(.75 - power);
                robot.lb.setPower(.75 + power);
                robot.rb.setPower(-.75 - power);
            }
        }

        robot.lf.setPower(0);
        robot.lb.setPower(0);
        robot.rf.setPower(0);
        robot.rb.setPower(0);

    }


    public void angularVelocity(double velocity) {
        PID Anglepid = new PID(0.01, 0.005, 0.0002);
        Anglepid.setLimits(-1,1);
        if (robot.LinearOpMode.opModeIsActive() && !robot.LinearOpMode.isStopRequested()) {
            double velocityError = robot.gyro.getAngularVelocity()-velocity;
            double power = Anglepid.getOutput(velocityError);
            robot.lf.setPower(power);
            robot.rf.setPower(-power);
            robot.lb.setPower(power);
            robot.rb.setPower(-power);
            robot.LinearOpMode.telemetry.addData("Angle: ", robot.gyro.getContinuosAngle());
            robot.LinearOpMode.telemetry.addData("Angular Velocity: ", robot.gyro.getAngularVelocity());
            robot.LinearOpMode.telemetry.update();
        }
    }
}


