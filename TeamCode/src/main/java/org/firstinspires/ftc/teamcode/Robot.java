package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.InvictaDrive;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.subsystems.imu.InvictaGyro;
import org.firstinspires.ftc.teamcode.subsystems.vision.InvictaVision;

/**
 * Robot.java - contains all subsystems, hardware, and vision software
 * @Author InvictaGary
 */
public class Robot {
    //Motors
    public DcMotor rf, rb, lf, lb, sweeper, winch, lift;

    //Servos
    public Servo flipper;
    public Servo marker;

    //Sensors
    public BNO055IMU imu;
    public TouchSensor slideLimit;
    public TouchSensor hookUp;
    public TouchSensor hookDown;
    //public VoltageSensor voltageSensor;

    //Subsystems
    public InvictaGyro gyro;
    public InvictaVision vision;
    public InvictaDrive drive;
    public OpMode opMode;
    public LinearOpMode LinearOpMode;
    public HardwareMap hardwareMap;

    //Parameters
    private VuforiaLocalizer.Parameters visionParams;

    public Robot(LinearOpMode l) {
        LinearOpMode = l;
        this.hardwareMap = LinearOpMode.hardwareMap;
        init();
    }

    public Robot(OpMode o) {
        opMode = o;
        this.hardwareMap = opMode.hardwareMap;
        init();
    }

    /**
     * Initializes all hardware and subsystems
     */
    private void init() {

        //Drive Motors
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");

        //Other Motors
        sweeper = hardwareMap.dcMotor.get("sweeper");
        winch = hardwareMap.dcMotor.get("winch");
        lift = hardwareMap.dcMotor.get("lift");

        //Motor Presets
        lf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        flipper = hardwareMap.servo.get("flipper");
        marker = hardwareMap.servo.get("marker");

        //Sensors (IMU + External)
        //voltageSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
        slideLimit = hardwareMap.touchSensor.get("slideLimit");
        hookDown = hardwareMap.touchSensor.get("hookDown");
        hookUp = hardwareMap.touchSensor.get("hookUp");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initImu();

        gyro = new InvictaGyro(this);
        drive = new InvictaDrive(this);

    }

    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //parameters object
        parameters.mode = BNO055IMU.SensorMode.IMU; //IMU sensormode, see documentation
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //Degrees as the unit for angle
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //M/s^2 as the unit for acceleration
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }

    private void initVision() {

        //API Key
        String VUFORIA_KEY = "AfYMsQX/////AAABmdgEbuGaNEvMtJgrGwPlsOR/Rt3/eMtBGeUwvJQM5dTQAd99U+cCjiu2/yvSqm6rvwJYa0fwX4k9tFRo/c+7b4ap11VKGmlo6aOxtnzNWmN9XB5+ykojHAwzgHy46mrvMnrO+IWl1PU3lOt2soBPr8OQuKvBV5C1ez2RXb86gYeWFiRs3JzBdWBU90QAFpYtnrREdMKjOiFOxH9zuA1DxemUg3iEwhF6irwzJzGFjFVvxOKCCoq4AMI+608L8qiz4Pb/CmYAN1Y+8XjMcgbkQpZ68cLCIHuLkcghj7lPpJJr1Ej99V9SGRY50z8kP3xsgMWiRqiUpKGPUyGtGJ9Ivh17Ogv1EKvLw7t9O7HxZcCD";

        //Loading in OpenCV so it works
        System.loadLibrary("opencv_java3");

        //The parameters for creating an instance of Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        visionParams = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        visionParams.vuforiaLicenseKey = VUFORIA_KEY;

        visionParams.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vision = new InvictaVision(visionParams);

        //We enable converting a Frame object to a Bitmap image
        vision.enableConvertFrameToBitmap();

        //We set the size of the BlockQueue of ClosedFrames to 1, because we only want the most recent image taken
        vision.setFrameQueueCapacity(1);

        //Loading in the 4 assets it needs to find and assigning names to them
        VuforiaTrackables targets = vision.loadTrackablesFromAsset("RoverRuckus");
        targets.get(0).setName("Blue-Rover");
        targets.get(1).setName("Red-Footprint");
        targets.get(2).setName("Front-Craters");
        targets.get(3).setName("Back-Space");
    }
}
