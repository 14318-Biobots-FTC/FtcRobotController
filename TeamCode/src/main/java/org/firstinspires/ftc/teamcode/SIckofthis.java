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
@Autonomous(name="sickofthis", group="Tutorials")

public class SIckofthis extends LinearOpMode {

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    double CrLowerUpdate = 0;
    double CbLowerUpdate = 150;
    double CrUpperUpdate = 150;
    double CbUpperUpdate = 255;

    double lowerruntime = 0;
    double upperruntime = 0;


    DcMotorEx leftFront;
    DcMotorEx rightRear ;
    DcMotorEx rightFront ;
    DcMotorEx leftRear ;
    CRServo rightDuck ;
    CRServo leftDuck ;
    CRServo capstone ;
    Servo stopper ;
    DcMotorEx intake ;
    DcMotor linearslideleft ;
    DcMotor linearslideright ;
    DcMotor actuator ;
    // blue Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0, 50);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120, 130);

    @Override
    public void runOpMode() throws InterruptedException
    {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline());
        // Configuration of Pipeline
        myPipeline.ConfigurePipeline(30, 30,30,30,  CAMERA_WIDTH, CAMERA_HEIGHT);
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

//            telemetry.addData("RectArea: ", myPipeline.getRectArea());
//            telemetry.addData("RectMidPoint: ", myPipeline.getRectMidpointX());
//            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
//                telemetry.addData("InLoopRectArea: ", myPipeline.getRectArea());
//                telemetry.addData("InLoopRectMidPoint: ", myPipeline.getRectMidpointX());
//                telemetry.update();
                if(myPipeline.getRectMidpointX() > 400){
                    AUTONOMOUS_C();
                }
                else if(myPipeline.getRectMidpointX() > 200){
                    AUTONOMOUS_B();
                }
                else {
//                    telemetry.addData("In else: ", "in else for Autonomous A");
//                    telemetry.update();
                    AUTONOMOUS_A();
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
//        telemetry.addLine("Autonomous A");
//        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        leftFront  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront  = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        // dw = hardwareMap.get(CRServo.class, "intake");
        leftDuck = hardwareMap.get(CRServo.class, "leftDuck");
        rightDuck = hardwareMap.get(CRServo.class, "rightDuck");
        capstone = hardwareMap.get(CRServo.class, "capstone");
        stopper = hardwareMap.get(Servo.class, "stopper");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

//        stopper = hardwareMap.get(Servo.class, "stopper");
        linearslideleft = hardwareMap.get(DcMotor.class, "linearslideleft");
        linearslideright = hardwareMap.get(DcMotor.class, "linearslideright");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
//        DcMotorEx intake=hardwareMap.get(DcMotorEx.class, "intake");
        int county=0;
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        linearslideleft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearslideright.setDirection(DcMotorSimple.Direction.REVERSE);
        linearslideleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearslideright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearslideleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearslideright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearslideleft.setTargetPosition(0);
        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setTargetPosition(0);
        linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setTargetPosition(0);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Pose2d start=new Pose2d(12,-60,Math.toRadians(0));
        drive.setPoseEstimate(start);
        Trajectory trajstart = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)))
                .addTemporalMarker(0,()->{
                    intake.setPower(-0.1);
                })
                .addTemporalMarker(0,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+2300);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+2300);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })
                .addTemporalMarker(0.9,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()+1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })

                .build();

        TrajectorySequence trajmain = drive.trajectorySequenceBuilder(trajstart.end())
                .splineTo(new Vector2d(20,-64),Math.toRadians(0))
                .forward(42)
                .back(39)
                .splineTo(new Vector2d(0,-34),Math.toRadians(90))
                //slow down intake
                .addTemporalMarker(4,()->{
                    intake.setPower(-0.4);
                })

                .addTemporalMarker(0,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })

                .addTemporalMarker(0.5,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-2300);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-2300);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })

                .addTemporalMarker(3.0,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+5500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+5500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })


                .addTemporalMarker(4.1,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()+1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })

                .build();
        int count=0;
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(start)
                .splineTo(new Vector2d(14,-66),Math.toRadians(-5))
//                .waitSeconds(0.2)
                .forward(45)
                .back(42)
                .splineTo(new Vector2d(0,-37),Math.toRadians(90))
                //slow down intake
                .addTemporalMarker(4,()->{
                    intake.setPower(-0.2);
                })
                .addTemporalMarker(0.2,()->{
                    stopper.setPosition(0);
                })
                .addTemporalMarker(0,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })

                .addTemporalMarker(0,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-5500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-5500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })

                .addTemporalMarker(3.2,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+5500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+5500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })


                .addTemporalMarker(4.65,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()+1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })

                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .splineTo(new Vector2d(14,-66  ),Math.toRadians(-5))
                .waitSeconds(0.3)
//                .waitSeconds(0.2)
                .forward(48)
                .back(45)
                .splineTo(new Vector2d(0,-37),Math.toRadians(90))
                //slow down intake
                .addTemporalMarker(4,()->{
                    intake.setPower(-0.2);
                })
                .addTemporalMarker(0.2,()->{
                    stopper.setPosition(0);
                })
                .addTemporalMarker(0,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })

                .addTemporalMarker(0.3,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-5500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-5500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })

                .addTemporalMarker(3.3,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+5500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+5500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })


                .addTemporalMarker(4.65,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()+1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })

                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .splineTo(new Vector2d(14,-60),Math.toRadians(-2))
                .waitSeconds(0.3)

                .forward(60)
                .addTemporalMarker(0,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })
                .addTemporalMarker(0.2,()->{
                    stopper.setPosition(0);
                })
                .addTemporalMarker(0.3,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-5500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-5500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })

                .build();
        drive.followTrajectory(trajstart);
        stopper.setPosition(1);
        sleep(300);
        intake.setPower(-1);
        sleep(500);
        stopper.setPosition(0);
        sleep(200);
        drive.followTrajectorySequence(trajmain);
        sleep(300);

        stopper.setPosition(1);
        sleep(300);
        intake.setPower(-1);
        sleep(540);
//        stopper.setPosition(0);
        //sleep(200);
        drive.followTrajectorySequence(traj2);
        sleep(300);

        stopper.setPosition(1);
        sleep(300);
        intake.setPower(-1);
        sleep(400);
//        stopper.setPosition(0);
        //sleep(200);
        drive.followTrajectorySequence(traj3);

        stopper.setPosition(1);
        sleep(300);
        intake.setPower(-1);
        sleep(400);
//        stopper.setPosition(0);
        //sleep(200);
        drive.followTrajectorySequence(traj4);


    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");
    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
    }
}