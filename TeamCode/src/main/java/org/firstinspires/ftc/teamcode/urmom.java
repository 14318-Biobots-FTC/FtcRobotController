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
@Autonomous(name="urmom", group="Tutorials")

public class urmom extends LinearOpMode {



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
        linearslideleft = hardwareMap.get(DcMotor.class, "linearslideleft");
        linearslideright = hardwareMap.get(DcMotor.class, "linearslideright");
        linearslideleft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearslideright.setDirection(DcMotorSimple.Direction.REVERSE);
        linearslideleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearslideright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearslideleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearslideright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stopper = hardwareMap.get(Servo.class, "stopper");
        stopper.setPosition(0);
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
                .lineToLinearHeading(new Pose2d(21, 36, Math.toRadians(40)))
                .addTemporalMarker(0,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+2400);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+2400);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideleft.setPower(1);
                    linearslideright.setPower(1);
                })
                .addTemporalMarker(1.2,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()+1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })
                .addTemporalMarker(3.3,()->{
                    stopper.setPosition(1);
                })

                .addTemporalMarker(3.5,()->{
                    intake.setPower(-0.7);
                })
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(30, 52, Math.toRadians(180)),Math.toRadians(270))
                .back(25)
                .addTemporalMarker(0.5,()->{
                    rightDuck.setPower(0.2);
                })
                .addTemporalMarker(0.5,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);                })
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(40)
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
                .splineToConstantHeading(new Vector2d(48,47),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-12,70),Math.toRadians(180))
                .addTemporalMarker(0,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-100);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })
                .addTemporalMarker(2.5,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-2400);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-2400);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })
                .addTemporalMarker(3.5,()->{
                    intake.setPower(-1);
                })
                .forward(53.5) //CHANGED FROM 10
                .build();
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
//                .forward(23)
//                .build();
//        Trajectory traj6 = drive.trajectoryBuilder(traj4.end())
//                .back(47)
//                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj4.end())
                .back(50)
                .splineToLinearHeading(new Pose2d(-3,44,Math.toRadians(120)),Math.toRadians(90))
                .addTemporalMarker(0.5,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+5500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+5500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);
                })
                .addTemporalMarker(2,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()+2200);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })
                .addTemporalMarker(4,()->{
                    stopper.setPosition(1);
                })
                .addTemporalMarker(4.6,()->{
                    intake.setPower(-1);

                })

                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-12,70,Math.toRadians(180)))
                //.forward(24) //CHANGED FROM 10
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(20)
                .build();

        //MOVE TO CAPSTONE
        drive.followTrajectory(traj);

        sleep(500);
        intake.setPower(0);
        stopper.setPosition(0);


        // GO TO DUCK WHEEL
        drive.followTrajectorySequence(traj2);
        sleep(1800);

        rightDuck.setPower(0);
        drive.followTrajectorySequence(traj4);

        intake.setPower(0);
        stopper.setPosition(0);

        drive.followTrajectorySequence(traj7);


        intake.setPower(0);
        stopper.setPosition(0);

        actuator.setTargetPosition(actuator.getCurrentPosition()-2200);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setPower(1);
        sleep(1000);
        //LINEAR SLIDE DOWN
        linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-5500);
        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-5500);
        linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setPower(1);
        linearslideleft.setPower(1);

        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);



    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx linearSlide=null;
        DcMotorEx flip=null;
        DcMotorEx intake = null;
        DcMotorEx flip2= null;

        CRServo dw = null;
        dw = hardwareMap.get(CRServo.class, "duckWheel");
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        flip = hardwareMap.get(DcMotorEx.class, "flip");
        flip.setDirection(DcMotorEx.Direction.REVERSE);
        flip2= hardwareMap.get(DcMotorEx.class, "flip2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start=new Pose2d(-36,60,Math.toRadians(0));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-26, 43, Math.toRadians(120)))
                .build();
        // swapping all the y's
        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineToLinearHeading(new Pose2d(-35, 56, Math.toRadians(180)), Math.toRadians(180))
                .build(); // 360 --> 180
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-60, 56, Math.toRadians(220)), Math.toRadians(220))
                .build();

//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(),true)
//                .splineToLinearHeading(new Pose2d(-54, -50, Math.toRadians(360)), Math.toRadians(270))
//                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .splineToLinearHeading(new Pose2d(12,-61,Math.toRadians(360)),Math.toRadians(270))
//                .build();
//        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
//                .splineTo(new Vector2d(12,-65),Math.toRadians(360))
//                .forward(10)
//                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(-35,56,Math.toRadians(180)),Math.toRadians(180))
                .build(); // 180
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(75) //CHANGED FROM 10
                .build();


        // go to deposit
        drive.followTrajectory(traj);
        //deposit
        //flip to the right angle
        flip.setTargetPosition(flip.getCurrentPosition()+90);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()+90);
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(1);
        flip2.setPower(1);
        sleep(1000);
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+1400);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
        sleep(900);

        //drop element
        intake.setPower(0.5);
        sleep(900);
        intake.setPower(0);
        //arm back ing
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-1400);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);

        // go to depot
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
//
        //turn duck wheel
        dw.setPower(1);
        sleep(4000);
        dw.setPower(0);
        // go to depot
        // drive.followTrajectorySequence(traj4); //Add marker
//        //move everything
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx linearSlide=null;
        DcMotorEx flip=null;
        DcMotorEx intake = null;
        DcMotorEx flip2= null;

        CRServo dw = null;
        dw = hardwareMap.get(CRServo.class, "duckWheel");
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        flip = hardwareMap.get(DcMotorEx.class, "flip");
        flip.setDirection(DcMotorEx.Direction.REVERSE);
        flip2= hardwareMap.get(DcMotorEx.class, "flip2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start=new Pose2d(-36,60,Math.toRadians(0));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-23, 40, Math.toRadians(120)))
                .build();
        // swapping all the y's
        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineToLinearHeading(new Pose2d(-35, 56, Math.toRadians(180)), Math.toRadians(180))
                .build(); // 360 --> 180
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-60, 56, Math.toRadians(220)), Math.toRadians(220))
                .build();

//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(),true)
//                .splineToLinearHeading(new Pose2d(-54, -50, Math.toRadians(360)), Math.toRadians(270))
//                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .splineToLinearHeading(new Pose2d(12,-61,Math.toRadians(360)),Math.toRadians(270))
//                .build();
//        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
//                .splineTo(new Vector2d(12,-65),Math.toRadians(360))
//                .forward(10)
//                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(-35,56,Math.toRadians(180)),Math.toRadians(180))
                .build(); // 180
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(75) //CHANGED FROM 10
                .build();

        // go to deposit
        drive.followTrajectory(traj);
        //deposit
        //flip to the right angle
        flip.setTargetPosition(flip.getCurrentPosition()+160);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()+160);
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(1);
        flip2.setPower(1);
        sleep(1000);
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+1500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
        sleep(900);

        //drop element
        intake.setPower(0.5);
        sleep(900);
        intake.setPower(0);
        //arm back ing
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-1500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);

        // go to depot
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
//
        //turn duck wheel
        dw.setPower(1);
        sleep(4000);
        dw.setPower(0);
        // go to depot
        // drive.followTrajectorySequence(traj4); //Add marker
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
    }
}