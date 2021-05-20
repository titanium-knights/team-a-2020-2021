package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.Intake
import org.firstinspires.ftc.teamcode.util.WobbleGoal
import java.lang.Math.toRadians

class RedSideAuto: LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        val wobbleGoal = WobbleGoal.standard(hardwareMap)

        val intake = Intake.standard(hardwareMap)
        val shooterMotor = hardwareMap.dcMotor["shooter1"]
        shooterMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val shooterMotor2 = hardwareMap.dcMotor["shooter2"]
        shooterMotor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val shooterServo = hardwareMap.servo["pinball"]
        val flap = hardwareMap.servo["Flap"]

        // init stuff
        wobbleGoal.liftArm()
        wobbleGoal.grab()
        flap.position = 0.00 //High goal: 0.00      PS: 0.05
        shooterServo.position = 0.00

        waitForStart()

        val box = 0 // TODO: Vision stuff

        val startPose = Pose2d(-64.0, -50.0, toRadians(180.0))
        drive.poseEstimate = startPose

        lateinit var trajectory: Trajectory

        when (box) {
            0 -> {
                trajectory = drive.trajectoryBuilder(startPose, toRadians(-90.0))
                        .splineToConstantHeading(Vector2d(-48.0, -62.0), toRadians(0.0))
                        .splineToConstantHeading(Vector2d(-48.0, 0.0), toRadians(0.0))
                        .build()
                drive.followTrajectory(trajectory)
            }
            1 -> {
                trajectory = drive.trajectoryBuilder(startPose, toRadians(-90.0))
                        .splineToConstantHeading(Vector2d(-48.0, -62.0), toRadians(0.0))
                        .splineToConstantHeading(Vector2d(-48.0, 0.0), toRadians(0.0))
                        .splineToConstantHeading(Vector2d(-36.0, 50.0), toRadians(90.0))
                        .build()
                drive.followTrajectory(trajectory)
            }
            2 -> {
                trajectory = drive.trajectoryBuilder(startPose, toRadians(-90.0))
                        .splineToConstantHeading(Vector2d(-48.0, -62.0), toRadians(0.0))
                        .splineToConstantHeading(Vector2d(-48.0, 50.0), toRadians(0.0))
                        .build()
                drive.followTrajectory(trajectory)
            }
        }

        val backup = drive.trajectoryBuilder(trajectory.end(), false)
                .splineToLinearHeading(Pose2d(-30.0, -26.0, toRadians(0.0)), toRadians(180.0)) //pick up wobble2
                .build()
        drive.followTrajectory(backup)
    }
}