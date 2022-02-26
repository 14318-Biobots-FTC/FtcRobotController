package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="FFAutoRedNearParkNear", group="Tutorials")

public class FFAutoRedNearParkNear extends LinearOpMode {



    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

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

        intake = hardwareMap.get(DcMotorEx.class, "intake");

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
        DcMotorEx flip2=null;
        DcMotorEx intake = null;

        CRServo dw = null;
        dw = hardwareMap.get(CRServo.class, "duckWheel");
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        flip = hardwareMap.get(DcMotorEx.class, "flip");
        flip.setDirection(DcMotorEx.Direction.REVERSE);
        flip2 = hardwareMap.get(DcMotorEx.class, "flip2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        flip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flip2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pose2d start=new Pose2d(12,-60,Math.toRadians(180));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(2, -46, Math.toRadians(290)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(12, -65, Math.toRadians(360)), Math.toRadians(360))
                .build();
        Trajectory mid = drive.trajectoryBuilder(traj2.end())
                .forward(30)
                .build();

        // go to deposit
        drive.followTrajectory(traj);
        //deposit
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+1500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
        sleep(900);
        intake.setPower(1);
        sleep(500);
        intake.setPower(0);
        //arm back ing
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-1500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);
        // go to depot
        drive.followTrajectory(traj2);
        drive.followTrajectory(mid);


    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx linearSlide=null;
        DcMotorEx flip=null;
        DcMotorEx flip2=null;
        DcMotorEx intake = null;

        CRServo dw = null;
        dw = hardwareMap.get(CRServo.class, "duckWheel");
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        flip = hardwareMap.get(DcMotorEx.class, "flip");
        flip.setDirection(DcMotorEx.Direction.REVERSE);
        flip2 = hardwareMap.get(DcMotorEx.class, "flip2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start=new Pose2d(12,-60,Math.toRadians(180));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(2, -43, Math.toRadians(300)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(12, -65, Math.toRadians(360)), Math.toRadians(360))
                .build();
        Trajectory mid = drive.trajectoryBuilder(traj2.end())
                .forward(30)
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
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+1500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
        sleep(900);
        intake.setPower(0.75);
        sleep(500);
        intake.setPower(0);
        //arm back ing
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-1500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);
        flip.setTargetPosition(flip.getCurrentPosition()-90);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()-90); //420
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(-1);
        flip2.setPower(-1);
        sleep(1000);
        // go to depot
        drive.followTrajectory(traj2);
        drive.followTrajectory(mid);


    }
    public void AUTONOMOUS_C(){
        //FROM B TO C: 90 --> 150
        telemetry.addLine("Autonomous C");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx linearSlide=null;
        DcMotorEx flip=null;
        DcMotorEx flip2=null;
        DcMotorEx intake = null;

        CRServo dw = null;
        dw = hardwareMap.get(CRServo.class, "duckWheel");
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        flip = hardwareMap.get(DcMotorEx.class, "flip");
        flip.setDirection(DcMotorEx.Direction.REVERSE);
        flip2 = hardwareMap.get(DcMotorEx.class, "flip2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start=new Pose2d(12,-60,Math.toRadians(180));
        drive.setPoseEstimate(start);
        Trajectory traj = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(2, -39, Math.toRadians(310)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(12, -60, Math.toRadians(360)), Math.toRadians(360))
                .build();
        Trajectory mid = drive.trajectoryBuilder(traj2.end())
                .forward(30)
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
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+1500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
        sleep(900);
        intake.setPower(0.4);
        sleep(500);
        intake.setPower(0);
        //arm back ing
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-1500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);
        sleep(300);
        flip.setTargetPosition(flip.getCurrentPosition()-160);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip2.setTargetPosition(flip2.getCurrentPosition()-160); //420
        flip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(-0.1);
        flip2.setPower(-0.1);
        sleep(1000);
        // go to depot
        drive.followTrajectory(traj2);
        drive.followTrajectory(mid);


    }
}