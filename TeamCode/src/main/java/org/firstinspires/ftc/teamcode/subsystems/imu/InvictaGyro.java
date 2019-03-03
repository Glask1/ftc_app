package org.firstinspires.ftc.teamcode.subsystems.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;

public class InvictaGyro {

    private BNO055IMU imu;
    private boolean isDisabled = false;
    private double oldAngle;
    private double correction = 0;
    private double correctedAngle = 0;

    private Thread workerThread;
    private Robot r;

    public InvictaGyro(Robot r) {
        this.r = r;
        this.imu = r.imu;
        if(workerThread == null || !workerThread.isAlive()) {
            start();
        }

    }

    public double getZAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private void start() {
        oldAngle = getZAngle();
        ElapsedTime time = new ElapsedTime();
        workerThread = new Thread(new Runnable() {
            @Override
            public void run() {
                ElapsedTime time = new ElapsedTime();
                double count = 0;
                while (!workerThread.isInterrupted() && !isDisabled && (r.opMode != null || !r.LinearOpMode.isStopRequested())) {
                    if ((getZAngle() - oldAngle) < -200) {
                        correction += 360;
                    } else if ((getZAngle() - oldAngle) > 200) {
                        correction -= 360;
                    }
                    correctedAngle = getZAngle() + correction;
                    oldAngle = getZAngle();
                    count++;
                }
                stop();

            }
        });
        workerThread.setName("InvictaImu");
        workerThread.start();
    }

    public void stop() {
        isDisabled = true;
        workerThread.interrupt();
    }

    public double getXAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
    }

    public double getYAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
    }
    public double getAngularVelocity() {
        return imu.getAngularVelocity().zRotationRate;
    }

    public double getContinuosAngle() {
        return correctedAngle;
    }

}
