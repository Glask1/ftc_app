package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.InvictaDrive;

@Autonomous(name="Drive Test")
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        waitForStart();
        //robot.drive.angularVelocity(90);
        ElapsedTime time = new ElapsedTime();
        double count = 0;
        double oldAngle = robot.gyro.getZAngle();
        double correction = 0;
        double correctedAngle = 0;
        while (!robot.LinearOpMode.isStopRequested()) {
            robot.LinearOpMode.telemetry.addData("fps", (count/time.seconds()));
            robot.LinearOpMode.telemetry.addData("angle", correctedAngle);
            robot.LinearOpMode.telemetry.addData("difference", robot.gyro.getZAngle() - oldAngle);
            robot.LinearOpMode.telemetry.update();
            if ((robot.gyro.getZAngle() - oldAngle) < -200) {
                correction += 360;
            } else if ((robot.gyro.getZAngle() - oldAngle) > 200) {
                correction -= 360;
            }
            correctedAngle = robot.gyro.getZAngle() + correction;
            oldAngle = robot.gyro.getZAngle();
            count++;
        }
        requestOpModeStop();
    }
}
