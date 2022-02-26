/*
This case is where we drop off an element, do the duck wheel, then go and park far away
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="FFAutoRedFar", group="Tutorials")

public class FFAutoRedFar extends LinearOpMode {



    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution 360

    double CrLowerUpdate = 0;
    double CbLowerUpdate = 150;
    double CrUpperUpdate = 150;
    double CbUpperUpdate = 255;

    double lowerruntime = 0;
    double upperruntime = 0;
    //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    DcMotorEx linearSlide=null;
    DcMotorEx flip=null;
    DcMotorEx intake = null;

    CRServo dw = null;
    // blue Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 150.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 150.0, 255.0);

    @Override
    public void runOpMode()
    {

        dw = hardwareMap.get(CRServo.class, "duckWheel");
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        flip = hardwareMap.get(DcMotorEx.class, "flip");

      //  intake = hardwareMap.get(CRServo.class, "intake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

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

        Pose2d start=new Pose2d(-36,-60,Math.toRadians(180));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-26, -35, Math.toRadians(240)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(360)), Math.toRadians(360))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(27)
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
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(-48,-47),Math.toRadians(360))
                .splineToConstantHeading(new Vector2d(12,-70),Math.toRadians(360))
                .forward(31) //CHANGED FROM 10
                .build();

        // go to deposit
        drive.followTrajectory(traj);
        //deposit
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+1600);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
        sleep(900);

        //drop element
        intake.setPower(0.5);
        sleep(900);
        intake.setPower(0);
        //arm back ing
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-1600);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);
        // go to depot
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        //turn duck wheel
        dw.setPower(-1);
        sleep(4000);
        dw.setPower(0);
        // go to depot
        drive.followTrajectorySequence(traj4); //Add marker
        //move everything

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

        Pose2d start=new Pose2d(-36,-60,Math.toRadians(180));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-26, -33, Math.toRadians(235)))
                .build();
//        Trajectory trajpre = drive.trajectoryBuilder(traj.end(), true)
//                .splineToLinearHeading(new Pose2d(-30, -45, Math.toRadians(360)), Math.toRadians(360))
//                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(360)), Math.toRadians(360))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(27)
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
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
                .splineToConstantHeading(new Vector2d(-48,-47),Math.toRadians(360))
                .splineToConstantHeading(new Vector2d(12,-70),Math.toRadians(360))
                .forward(30) //CHANGED FROM 10
                .build();

        // go to deposit
        drive.followTrajectory(traj);
        //deposit
        flip.setTargetPosition(flip.getCurrentPosition()+90);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()+90); //420
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(1);
        flip2.setPower(1);
        sleep(1000);

        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+1700);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
        sleep(1000);

        //drop element
        intake.setPower(0.5);
        sleep(1000);
        intake.setPower(0);
        flip.setTargetPosition(flip.getCurrentPosition()+30);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()+30); //420
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(1);
        flip2.setPower(1);
        sleep(200);
        //arm back ing
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-1700);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);
        // go to depot
        sleep(200);
        flip.setTargetPosition(flip.getCurrentPosition()-90);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()-90); //420
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(-1);
        flip2.setPower(-1);
//        drive.followTrajectory(trajpre);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        //turn duck wheel
        dw.setPower(-1);
        sleep(4000);
        dw.setPower(0);
        // go to depot
        drive.followTrajectorySequence(traj4); //Add marker
        //move everything

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

        Pose2d start=new Pose2d(-36,-60,Math.toRadians(180));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-26, -31, Math.toRadians(235)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineToLinearHeading(new Pose2d(-35, -56, Math.toRadians(360)), Math.toRadians(360))
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
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj2.end())
                .splineToConstantHeading(new Vector2d(-48,-47),Math.toRadians(360))
                .splineToConstantHeading(new Vector2d(12,-70),Math.toRadians(360))
                .forward(30) //CHANGED FROM 10
                .build();

        // go to deposit
        drive.followTrajectory(traj);
        //deposit
        flip.setTargetPosition(flip.getCurrentPosition()+160);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()+160); //420
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(1);
        flip2.setPower(1);
        sleep(1000);

        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+1700);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
        sleep(1000);

        //drop element
        intake.setPower(0.5);
        sleep(1000);
        intake.setPower(0);
        flip.setTargetPosition(flip.getCurrentPosition()+30);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()+30); //420
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(1);
        flip2.setPower(1);
        //arm back ing
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-1700);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);
        // go to depot
        sleep(200);
        flip.setTargetPosition(flip.getCurrentPosition()-160);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()-160); //420
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(-1);
        flip2.setPower(-1);
        drive.followTrajectory(traj2);
        //turn duck wheel
        dw.setPower(-1);
        sleep(4000);
        dw.setPower(0);
        // go to depot
        drive.followTrajectorySequence(traj4); //Add marker
        //move everything

    }
}