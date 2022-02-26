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
@Autonomous(name="TestAuto", group="Tutorials")

public class TestAuto extends LinearOpMode {



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
    DcMotor actuator = null;
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
        //telemetry.update();

        telemetry.addLine("Autonomous A");
        //telemetry.update();

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

        Pose2d start=new Pose2d(36,60,Math.toRadians(180));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-0, -0, Math.toRadians(220)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineToLinearHeading(new Pose2d(-62, -56, Math.toRadians(360)), Math.toRadians(270))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(),true)
                .splineToLinearHeading(new Pose2d(-54, -50, Math.toRadians(360)), Math.toRadians(270))
                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .splineToLinearHeading(new Pose2d(12,-61,Math.toRadians(360)),Math.toRadians(270))
//                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
                .splineTo(new Vector2d(12,-63),Math.toRadians(360))
                .forward(20)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(10)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .back(30)
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToLinearHeading(new Pose2d(2, -46, Math.toRadians(300)))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .splineToLinearHeading(new Pose2d(12, -63, Math.toRadians(360)), Math.toRadians(360))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(30)
                .build();
        // go to deposit
        drive.followTrajectory(traj);
        //deposit
        linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+1000);
        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+1000);
        linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setPower(1);
        linearslideleft.setPower(1);
        sleep(1500);

        //MOVE ACTUATOR OUT
//        actuator.setTargetPosition(actuator.getCurrentPosition()+400);
//        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        actuator.setPower(1);

        //drop element
        sleep(900);
        stopper.setPosition(1);
        intake.setPower(0.5);
        sleep(900);
        intake.setPower(0);
        stopper.setPosition(0);

        //ACTUATOR IN
//        actuator.setTargetPosition(actuator.getCurrentPosition()-400);
//        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        actuator.setPower(1);

        //LINEAR SLIDE DOWN
        linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-1000);
        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-1000   );
        linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setPower(1);
        linearslideleft.setPower(1);



//        drive.followTrajectory(traj2);
//        dw.setPower(-1);
//        sleep(4000);
//        dw.setPower(0);
//        //drive.followTrajectory(traj3);
//        drive.followTrajectorySequence(traj4);
//        //drive.followTrajectory(traj5);
//        flip.setTargetPosition(flip.getCurrentPosition()+370);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setPower(1);
//        sleep(200);
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-300);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(-1);
//        sleep(200);
//        intake.setPower(-1);
//        drive.followTrajectory(traj5);
//
//        sleep(1000);
//        intake.setPower(0);
//
//        flip.setTargetPosition(flip.getCurrentPosition()-300);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setPower(-1);
//        drive.followTrajectory(traj6);
//        drive.followTrajectory(traj7);
//
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+750);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(1);
//        sleep(700);
//        intake.setPower(-1);
//        sleep(900);
//        intake.setPower(0);
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-750);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(-1);
//
//        drive.followTrajectory(traj8);
//        drive.followTrajectory(traj9);


    }
    public void AUTONOMOUS_B(){
//        telemetry.addLine("Autonomous B");
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        DcMotorEx linearSlide=null;
//        DcMotorEx flip=null;
//        CRServo intake = null;
//
//        CRServo dw = null;
//        dw = hardwareMap.get(CRServo.class, "duckWheel");
//        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
//        flip = hardwareMap.get(DcMotorEx.class, "flip");
//
//        intake = hardwareMap.get(CRServo.class, "intake");
//        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d start=new Pose2d(-36,-60,Math.toRadians(180));
//        drive.setPoseEstimate(start);
//        Trajectory traj = drive.trajectoryBuilder(start)
//                .lineToLinearHeading(new Pose2d(-27, -43, Math.toRadians(240)))
//                .build();
//        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
//                .splineToLinearHeading(new Pose2d(-60, -57, Math.toRadians(330)), Math.toRadians(240))
//                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .splineToConstantHeading(new Vector2d(-54, -50), Math.toRadians(330))
//                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .splineToLinearHeading(new Pose2d(12,-61,Math.toRadians(360)),Math.toRadians(270))
//                .build();
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
//                .forward(30)
//                .build();
//        // go to deposit
//        drive.followTrajectory(traj);
//        //deposit
//        flip.setTargetPosition(flip.getCurrentPosition()+150);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setPower(1);
//        sleep(1000);
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+700);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(1);
//        sleep(200);
//        intake.setPower(-1);
//        sleep(1500);
//        intake.setPower(0);
////        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+600);
////        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        linearSlide.setPower(1);
//        //arm back ing
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-700);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(-1);
//        // go to depot
//        sleep(200);
//        flip.setTargetPosition(flip.getCurrentPosition()-150);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setPower(-1);
//        sleep(1000);
//        drive.followTrajectory(traj2);
//        dw.setPower(-1);
//        sleep(3500);
//        dw.setPower(0);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        flip.setTargetPosition(flip.getCurrentPosition()+400);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setPower(1);
//        sleep(500);
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-100);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(-1);
//        sleep(200);
//        drive.followTrajectory(traj5);
    }
    public void AUTONOMOUS_C(){
//        telemetry.addLine("Autonomous C");
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        DcMotorEx linearSlide=null;
//        DcMotorEx flip=null;
//        CRServo intake = null;
//
//        CRServo dw = null;
//        dw = hardwareMap.get(CRServo.class, "duckWheel");
//        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
//        flip = hardwareMap.get(DcMotorEx.class, "flip");
//
//        intake = hardwareMap.get(CRServo.class, "intake");
//        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d start=new Pose2d(-36,-60,Math.toRadians(180));
//        drive.setPoseEstimate(start);
//        Trajectory traj = drive.trajectoryBuilder(start)
//                .lineToLinearHeading(new Pose2d(-27, -38, Math.toRadians(240)))
//                .build();
//        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
//                .splineToLinearHeading(new Pose2d(-60, -57, Math.toRadians(330)), Math.toRadians(240))
//                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .splineToConstantHeading(new Vector2d(-54, -50), Math.toRadians(330))
//                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .splineToLinearHeading(new Pose2d(12,-61,Math.toRadians(360)),Math.toRadians(270))
//                .build();
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
//                .forward(30)
//                .build();
//        // go to deposit
//        drive.followTrajectory(traj);
//        //deposit
//        flip.setTargetPosition(flip.getCurrentPosition()+225);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setPower(1);
//        sleep(1000);
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+700);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(1);
//        sleep(200);
//        intake.setPower(-1);
//        sleep(1500);
//        intake.setPower(0);
////        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+600);
////        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        linearSlide.setPower(1);
//        //arm back ing
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-700);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(-1);
//        // go to depot
//        sleep(200);
//        flip.setTargetPosition(flip.getCurrentPosition()-225);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setPower(-1);
//        sleep(1000);
//        drive.followTrajectory(traj2);
//        dw.setPower(-1);
//        sleep(3500);
//        dw.setPower(0);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        flip.setTargetPosition(flip.getCurrentPosition()+400);
//        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flip.setPower(1);
//        sleep(500);
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-100);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(-1);
//        sleep(200);
//        drive.followTrajectory(traj5);
    }
}