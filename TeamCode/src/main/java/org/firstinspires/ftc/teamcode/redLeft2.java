package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class redLeft2 extends LinearOpMode {
    private DcMotor armMotor = null;
    private DcMotor liftMotor = null;

    private CRServo Claw = null;
    private Servo Turn = null;


    public static final int ARM_COUNTS_PER_MOTOR_REV = 1996;
    public static final double ARM_DEGREES_PER_COUNT = 360.0 / ARM_COUNTS_PER_MOTOR_REV;

    public double armTargetAngle = 65.0;

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Waiting for Start");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        Claw.setPower(-0.4);

        waitForStart();
        if (opModeIsActive()) {

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
                    .addTemporalMarker(9,()->{
                        liftMotor.setPower(-0.6);
                     })
                    .back(23.5)
                    .addTemporalMarker(10.2,()->{
                        liftMotor.setPower(0);
                        Claw.setPower(0);
                    })
                    .back(0.5)
                    .waitSeconds(2)
                   /* .lineToLinearHeading(new Pose2d(23.5,-100,Math.toRadians(-4)))
                    .addDisplacementMarker(()->{
                        setArmPosition(13.5);
                    })
                    .forward(0.8)
                    .addDisplacementMarker(()->{
                        setArmPosition(165);
                    })
                    .back(7.5)
                    .waitSeconds(2)
                    .addDisplacementMarker(()->{
                        setArmPosition(90);
                    })
                    .lineToLinearHeading(new Pose2d(23.5,-110))
                    .waitSeconds(2)

                    */
                    .build();



            drive.followTrajectorySequence(traj);
            Claw.setPower(0.4);
            sleep(100);
            Claw.setPower(0);

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
        armMotor.setPower(0.3);
    }

}
