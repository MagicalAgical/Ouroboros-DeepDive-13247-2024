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
public class Right1 extends LinearOpMode {
    private DcMotor armMotor = null;
    private DcMotor liftMotor = null;

    private CRServo Claw = null;
    private Servo Turn = null;


    public static final int ARM_COUNTS_PER_MOTOR_REV = 1996;
    public static final double ARM_DEGREES_PER_COUNT = 360.0 / ARM_COUNTS_PER_MOTOR_REV;

    public double armTargetAngle = 68.0;
    @Override
    public void runOpMode() throws InterruptedException {




        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift");

        Claw = hardwareMap.get(CRServo.class, "Claw");
        Turn = hardwareMap.get(Servo.class, "Turn");

        armMotor.setTargetPosition(360);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setArmPosition(armTargetAngle);
        Turn.setPosition(0.01);

        telemetry.addLine("Waiting for Start");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0,4,Math.toRadians(0)))
                .forward(35)
                .waitSeconds(1)
                .back(10)
                .waitSeconds(1)
                .strafeRight(60)
                .waitSeconds(1)
                .addDisplacementMarker(()->{
                    setArmPosition(18);
                })
                .waitSeconds(1)
                .back(11)
                .waitSeconds(1)
                .build();
        waitForStart();

        if (opModeIsActive()) {
            drive.followTrajectorySequence(traj);

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

