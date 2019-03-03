package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.vision.InvictaCV;
import org.firstinspires.ftc.teamcode.subsystems.vision.InvictaCVHull;
import org.invictarobotics.invictavision.CameraViewDisplay;

@Autonomous(name = "OpenCV2 only")
@Disabled
public class OpenCV2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        InvictaCVHull invictaCV = new InvictaCVHull();
        invictaCV.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        invictaCV.enable();


        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("fps", invictaCV.fps);
            telemetry.update();
        }

        invictaCV.disable();
    }
}
