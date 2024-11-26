package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// scoring x cord =
// scoring y cord =

//taking brick x cord = 27
@Autonomous
public class blueLeft extends LinearOpMode {

    private DcMotor armMotor = null;
    private DcMotor liftMotor = null;

    private CRServo Claw = null;
    private Servo Turn = null;


    public static final int ARM_COUNTS_PER_MOTOR_REV = 1996;
    public static final double ARM_DEGREES_PER_COUNT = 360.0 / ARM_COUNTS_PER_MOTOR_REV;

    public double armTargetAngle = 90.0;

    @Override
    public void runOpMode() throws InterruptedException {



        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift");

        Claw = hardwareMap.get(CRServo.class, "Claw");
        Turn = hardwareMap.get(Servo.class, "Turn");

        armMotor.setTargetPosition(360);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setArmPosition(armTargetAngle);
        Turn.setPosition(0.01);


        Claw.setPower(-0.3);

        telemetry.addLine("Waiting for Start");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    Claw.setPower(-0.3);
                })
                .forward(23)
                .turn(Math.toRadians(-45))
                .addDisplacementMarker(()->{
                    setArmPosition(100);
                    Turn.setPosition(0.3);
                })
                .addTemporalMarker(10,()->{
                    liftMotor.setPower(-0.5);
                })
                .back(23.5)
                .addTemporalMarker(10.2,()->{
                    liftMotor.setPower(0);
                    Claw.setPower(0);
                })
                .addTemporalMarker(10.2,()->{
                    Claw.setPower(0.3);
                })
                .addTemporalMarker(10.3,()->{
                    Claw.setPower(0);
                })
                .back(0.5)
                .waitSeconds(2)
                .build();

    /*   TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(-48)))
                .lineToLinearHeading(new Pose2d(15,4,Math.toRadians(10)))
                .waitSeconds(0.5)// marker to grab block
               /* .back(10)
                .lineToLinearHeading(new Pose2d(2,-2,Math.toRadians(-48)))
                .back(12)
                // add marker here
                .forward(2)
                .turn(Math.toRadians(48))
                .lineTo(new Vector2d())
                 .build();
                */




        waitForStart();
        if (opModeIsActive()) {
            //setArmPosition(65)
            drive.followTrajectorySequence(traj);
            /* setLiftPosition(-325);
            Turn.setPosition(0.08);

             */

            // drive.followTrajectorySequence(traj2);


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
    public void setArmPosition(double angle) {
        int targetPosition = (int) (angle / ARM_DEGREES_PER_COUNT);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(0.2);
    }

    public void setLiftPosition (double motorTargetPos) {
        int value = (int) motorTargetPos;
        liftMotor.setTargetPosition(value);
        liftMotor.setPower(0.5);
    }

}