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
@Autonomous(name="depot", group="Tutorials")

public class depot extends LinearOpMode {



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
                    sleep(3000);
                    AUTONOMOUS_C();
                    break;
                }
                else if(myPipeline.getRectMidpointX() > 200){
                    sleep(3000);
                    AUTONOMOUS_B();
                    break;
                }
                else {
                    sleep(3000);
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
                .lineToLinearHeading(new Pose2d(23, 34, Math.toRadians(45)))
                .addTemporalMarker(0,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+2500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+2500);
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
                .splineToLinearHeading(new Pose2d(30, 53, Math.toRadians(180)),Math.toRadians(270)) // 32 --> 30
                .back(25)
                .addTemporalMarker(0.5,()->{
                    rightDuck.setPower(0.6);
                })
                .addTemporalMarker(0.5,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                    })
                .addTemporalMarker(1.0,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-1700);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-1700);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);})
                .build();

//

        //MOVE TO CAPSTONE
        //sleep(5000);
        drive.followTrajectory(traj);

        sleep(1000);
        intake.setPower(0);
        stopper.setPosition(0);


        // GO TO DUCK WHEEL
        drive.followTrajectorySequence(traj2);
        sleep(5000);

        rightDuck.setPower(0);
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj2.end())
                .strafeLeft(20)
                .back(4)
                .build();
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .back(5)
//                .build();
        drive.followTrajectorySequence(traj5);
//        drive.followTrajectory(traj6);
        // drive.followTrajectorySequence(traj4);

//        intake.setPower(0);
//        stopper.setPosition(0);

        linearslideleft.setTargetPosition(0);
        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setTargetPosition(0);
        linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setPower(1);
        linearslideleft.setPower(1);




    }
    public void AUTONOMOUS_B(){

        telemetry.addLine("Autonomous B");
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
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+4000);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+4000);
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
                .splineToLinearHeading(new Pose2d(32, 51.5, Math.toRadians(180)),Math.toRadians(270))
                .back(25)
                .addTemporalMarker(0.5,()->{
                    rightDuck.setPower(0.6);
                })
                .addTemporalMarker(0.5,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-1700);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })
                .addTemporalMarker(1.2,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-3500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-3500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);})
                .build();

//

        //MOVE TO CAPSTONE
        drive.followTrajectory(traj);

        sleep(1000);
        intake.setPower(0);
        stopper.setPosition(0);


        // GO TO DUCK WHEEL
        drive.followTrajectorySequence(traj2);
        sleep(5000);

        rightDuck.setPower(0);
       // TrajectorySequence traj5 = drive.TrajectorySequenceBuilder(traj2.end())
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj2.end())
                .strafeLeft(22)
                .back(4)
                .build();
        // Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .back(5)
//                .build();
        drive.followTrajectorySequence(traj5);
        linearslideleft.setTargetPosition(0);
        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setTargetPosition(0);
        linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setPower(1);
        linearslideleft.setPower(1);
    }
    public void AUTONOMOUS_C(){

        telemetry.addLine("Autonomous C");
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
                .lineToLinearHeading(new Pose2d(21, 35, Math.toRadians(40)))
                .addTemporalMarker(0,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()+7000);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()+7000);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideleft.setPower(1);
                    linearslideright.setPower(1);
                })
                .addTemporalMarker(1.5,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()+1900);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })
                .addTemporalMarker(3.9,()->{
                    stopper.setPosition(1);
                })

                .addTemporalMarker(4.1,()->{
                    intake.setPower(-0.7);
                })
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(32, 51.0, Math.toRadians(180)),Math.toRadians(270))
                .back(25)
                .addTemporalMarker(0.5,()->{
                    rightDuck.setPower(0.6);
                })
                .addTemporalMarker(0.5,()->{
                    actuator.setTargetPosition(actuator.getCurrentPosition()-1900);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(1);
                })
                .addTemporalMarker(1.0,()->{
                    linearslideleft.setTargetPosition(linearslideleft.getCurrentPosition()-6500);
                    linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setTargetPosition(linearslideright.getCurrentPosition()-6500);
                    linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearslideright.setPower(1);
                    linearslideleft.setPower(1);})
                .build();

//

        //MOVE TO CAPSTONE
        drive.followTrajectory(traj);

        sleep(2000);
        intake.setPower(0);
        stopper.setPosition(0);


        // GO TO DUCK WHEEL
        drive.followTrajectorySequence(traj2);
        sleep(5000);

        rightDuck.setPower(0);
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj2.end())
                .strafeLeft(22)
                .back(4)
                .build();
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .back(5)
//                .build();
        drive.followTrajectorySequence(traj5);
        linearslideleft.setTargetPosition(0);
        linearslideleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setTargetPosition(0);
        linearslideright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslideright.setPower(1);
        linearslideleft.setPower(1);

    }
}