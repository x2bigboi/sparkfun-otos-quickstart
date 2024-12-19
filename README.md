# Downloading the quickstart as a new project

Download the Pinpoint branch of https://github.com/jdhs-ftc/sparkfun-otos-quickstart/tree/pinpoint
or `git clone -b pinpoint https://github.com/jdhs-ftc/sparkfun-otos-quickstart.git`

# Adding to an existing project (with git)

Easiest way to copy over all the changes to an existing repo is to run these 2 commands:

`git remote add pinpoint-quickstart https://github.com/jdhs-ftc/sparkfun-otos-quickstart`
`git pull pinpoint-quickstart pinpoint --no-rebase`

It is *possible* to copy everything over manually but would not recommend, it is kind of a pain.

(If the git method doesn't work, it would be easier to copy your teamcode files to a new downloaded copy of my quickstart)

# Tuning

Tuning steps are mostly the same as https://rr.brott.dev/docs/v1-0/tuning/

Differences:

Change the drive class in TuningOpModes, SplineTest, and your op modes to PinpointDrive

Ensure you configure the Pinpoint in your hardwaremap/DS config as `pinpoint` 
(or change the name in PinpointDrive)

Leave inPerTick at 1.0 in MecanumDrive, and leave localizer as DriveLocalizer (or set it to `null`).

Set your encoder resolution in PinpointDrive to one of the presets for gobilda odometry.
If you're not using gobilda odometry you will have to tune it manually. 

Use DeadWheelDirectionDebugger to properly reverse your dead wheels in PinpointDrive.

Measure your odometry offsets **manually** (with a tape measure).
Note that AngularRampLogger cannot automatically tune them.

The X pod offset refers to how far **sideways** from the center the X (forward) odometry pod is.
Left of the center is a positive number, right of the center is a negative number.
The Y pod offset refers to how far **forwards** from the center the Y (strafe) odometry pod is:
forward of the center is a positive number, backwards is a negative number.

Your offsets don't need to be perfect.
Any error will show up as the robot orbiting around a point on FTC Dashboard when spinning in place IRL.

Finally, you can begin standard tuning on https://rr.brott.dev/docs/v1-0/tuning/ but make sure to **skip ForwardPushTest.**

Some teams have reported issues tuning trackwidth with AngularRampLogger as well. It may be easier
to measure that manually.

If you use this, please let me know how it goes, both if things go wrong but *especially* if it all
works smoothly; I'd love more people to test this before moving forward with a more formal
release/community news post.