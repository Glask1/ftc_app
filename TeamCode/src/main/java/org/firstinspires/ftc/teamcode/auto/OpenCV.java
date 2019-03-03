package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.vision.InvictaCV;
import org.invictarobotics.invictavision.CameraViewDisplay;
@Disabled
//@Autonomous(name = "OpenCV only")
public class OpenCV extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        InvictaCV invictaCV = new InvictaCV();
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
