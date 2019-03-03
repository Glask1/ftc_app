package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.InvictaCVHull;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.invictarobotics.invictavision.CameraViewDisplay;

@Autonomous(name="pidMove")
@Disabled
public class VisionTestOnly extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot robot = new Robot(this);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        double kp = 0.1,
                ki = 0,
                kd = 0;
        int column = 0;

        double error;
        ElapsedTime timeout = new ElapsedTime();

        while(opModeIsActive()) {

            /*if(gamepad1.dpad_right && column < 2 && timeout.seconds() > .1) {
                timeout.reset();
                column += 1;
            } else if(gamepad1.dpad_left && column > 0 && timeout.seconds() > .1) {
                column -= 1;
                timeout.reset();
            }
            if(gamepad1.dpad_up && timeout.seconds() > .1) {
                timeout.reset();
                if(column == 0) {
                    kp += .001;
                }
                else if(column == 1 ) {
                    ki += .001;
                }
                else if(column == 2) {
                    kd += .001;
                }
            } else if(gamepad1.dpad_down && timeout.seconds() > .1) {
                timeout.reset();
                if(column == 0) {
                    kp -= .001;
                }
                else if(column == 1) {
                    ki -= .001;
                }
                else if(column == 2) {
                    kd -= .001;
                }
            }*/

            robot.drive.move2(30);
            sleep(10000);
            break;
        }
        robot.gyro.stop();

    }
}
