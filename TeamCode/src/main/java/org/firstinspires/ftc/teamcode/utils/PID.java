package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PID {

    private double kP, kI, kD;

    private double errorSum = 0;
    private double lastTime = 0;
    private double lastError = 0;
    private double min, max;

    private boolean firstTime;
    private boolean limited = false;

    public PID(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
        reset();
    }

    public void setValues(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    private void reset() {
        firstTime = true;
    }

    public double getOutput(double error) {

        double time = System.nanoTime() / Math.pow(10, 9);

        double output = 0;

        //Proportional
        output += kP * error;

        if (firstTime) {
            errorSum = 0;
            firstTime = false;
        } else {
            double deltaTime = time - lastTime;

            //Integral
            errorSum += (error + lastError) * deltaTime/2;
            output += kI * errorSum;

            //Derivative
            output += kD * (error - lastError) / deltaTime;
        }

        lastError = error;
        lastTime = time;

        if(limited) {
            output = Range.clip(output, min, max);
        }

        return output;
    }

    public void setLimits(double low, double high) {
        min = low;
        max = high;
        limited = true;
    }

}
