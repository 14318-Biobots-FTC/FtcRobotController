/*
This case is where we drop off an element, do the duck wheel, then go and park along the wall
(Except on the blue side instead of red side this time.)
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="cyclerednew", group="Tutorials")

public class Cyclerednew extends LinearOpMode {



    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution 360

    double CrLowerUpdate = 0;
    double CbLowerUpdate = 0;
    double CrUpperUpdate = 150;
    double CbUpperUpdate = 150;

    double lowerruntime = 0;
    double upperruntime = 0;
    //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    DcMotorEx leftFront = null;
    DcMotorEx rightRear = null;
    DcMotorEx rightFront = null;
    DcMotorEx leftRear = null;
    CRServo rightDuck = null;
    CRServo leftDuck = null;
    CRServo capstone = null;
    Servo stopper = null;
    DcMotorEx intake = null;
    DcMotor linearslideleft = null;
    DcMotor linearslideright = null;
    // blue Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0, 50);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120, 130);

    @Override
    public void runOpMode()
    {



        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline());
        // Configuration of Pipeline
//        myPipeline.ConfigurePipeline(30, 30,30,30,  CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigurePipeline(0, 0,0,0,  CAMERA_WIDTH, CAMERA_HEIGHT);

        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });



        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
            //testing(myPipeline);

            // Watch our YouTube Tutorial for the better explanation

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 400){
                    AUTONOMOUS_C();
                    break;
                }
                else if(myPipeline.getRectMidpointX() > 200){
                    AUTONOMOUS_B();
                    break;
                }
                else {
                    AUTONOMOUS_A();
                    break;
                }
            }
        }
    }
    public void testing(ContourPipeline myPipeline){
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.ConfigureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.ConfigureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        leftFront  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront  = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        // dw = hardwareMap.get(CRServo.class, "intake");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

//        stopper = hardwareMap.get(Servo.class, "stopper");
        linearslideleft = hardwareMap.get(DcMotor.class, "linearslideleft");
        linearslideright = hardwareMap.get(DcMotor.class, "linearslideright");
//        DcMotorEx intake=hardwareMap.get(DcMotorEx.class, "intake");
        int county=0;
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
//        linearslideleft.setDirection(DcMotorSimple.Direction.REVERSE);
//        linearslideright.setDirection(DcMotorSimple.Direction.REVERSE);
//        linearslideleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linearslideright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linearslideleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearslideright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearslideleft.setTargetPosition(0);
//        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearslideright.setTargetPosition(0);
//        linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        Pose2d start=new Pose2d(12,-60,Math.toRadians(-90));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(2, -40, Math.toRadians(-50)))
//                .addTemporalMarker(0,()->{
//                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+2600); // 2500
//                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+2600); // 2500
//                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    linearslideright.setPower(1);
//                    linearslideleft.setPower(1);
//                })

                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(traj.end())
                .splineTo(new Vector2d(24,-63),Math.toRadians(0))
                .forward(25)
                .back(25)
                .splineTo(new Vector2d(-2,-40),Math.toRadians(130))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(24,-62),Math.toRadians(0))
                .forward(25)
                .back(25)
                .splineTo(new Vector2d(-2,-40),Math.toRadians(130))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(24,-62.5),Math.toRadians(0))
                .forward(25)
                .back(25)
                .splineTo(new Vector2d(-2,-40),Math.toRadians(130))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(20,-63),Math.toRadians(0))
                .forward(25)
                .back(25)
                .splineTo(new Vector2d(-2,-40),Math.toRadians(130))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(20,-63.5),Math.toRadians(0))
                .forward(25)
                .back(25)
                .splineTo(new Vector2d(-2,-40),Math.toRadians(130))
                .splineTo(new Vector2d(20,-64),Math.toRadians(0))
                .forward(25)
                .build();
        drive.followTrajectory(traj);
        drive.followTrajectorySequence(cycle);
    }
    public void AUTONOMOUS_B(){

    }
    public void AUTONOMOUS_C(){

    }
}