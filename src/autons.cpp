#include "main.h"
#include "pros/gps.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 100;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  // chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_drive_constants_set(12.0, 0.0, 5.0);         // Fwd/rev constants, used for odom and non odom motions

  // chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_heading_constants_set(10.5, 0.0, 9.50);        // Holds the robot straight while going forward without odom

  chassis.pid_turn_constants_set(3.0, 0.05, 5.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  // chassis.pid_heading_constants_set(0.0, 0.0, 0.0);  // Disables the correction

  chassis.pid_drive_set(24_in, 127, false);
  chassis.pid_wait();

  // chassis.pid_drive_set(-24_in, 127);
  // chassis.pid_wait();f


  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait();

  // chassis.pid_turn_set(45_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();
  
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.ظظظ
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  // chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // chassis.pid_turn_set(45_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  // chassis.pid_wait();

  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
    const double side_length = 24.0; // in inches
   double turn_angle = 90.0;  // degrees (right turn)

  for (int i = 0; i < 4; i++) {
    // Drive forward one side of the square
    chassis.pid_drive_set(side_length * 1_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    // Turn 90 degrees clockwise
    chassis.pid_turn_set(turn_angle * 1_deg , TURN_SPEED);
    chassis.pid_wait();
    turn_angle += 90;
  }
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  // chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  // chassis.pid_odom_set({{{0_in, 48_in}, fwd, DRIVE_SPEED},
  //                       {{34_in, 34_in}, fwd, DRIVE_SPEED},
  //                       {{0_in, 68_in}, fwd, DRIVE_SPEED}},
  //                      true);
  chassis.pid_odom_set({{{17_in, 17_in}, fwd, DRIVE_SPEED},
                      {{34_in, 34_in}, fwd, DRIVE_SPEED},
                      {{0_in, 68_in}, fwd, DRIVE_SPEED}},
                     true);
  // chassis.pid_wait();
 chassis.pid_wait_until_index(2);  // Waits until the robot passes 12, 24
  intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  pros::delay(3000);    
    // chassis.pid_wait();              // Wait until final point (0, 68) is reached
  // Drive to 0, 0 backwards
  // chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
  //                      true);
  // chassis.pid_wait();
    chassis.pid_odom_set({
    {{10_in, 34_in}, rev, DRIVE_SPEED},  // Curve to the right
    {{0_in, 0_in}, rev, DRIVE_SPEED}
  }, true);
  chassis.pid_wait();

}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
    // chassis.pid_turn_set(45, 100);

  chassis.pid_odom_set({{{0_in, 48_in}, fwd, DRIVE_SPEED},
                        {{-48_in, 0_in}, fwd, DRIVE_SPEED},
                        {{0_in, 12_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  intake.move(0);  // Turn the intake off
}
///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, -24_in, 45_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .
void infinity_path_example() {
  // First loop (left curve of ∞)
  chassis.pid_odom_set({
    {{12_in, 12_in}, fwd, DRIVE_SPEED},
    {{0_in, 24_in}, fwd, DRIVE_SPEED},
    {{-12_in, 12_in}, fwd, DRIVE_SPEED}
  }, true);
  chassis.pid_wait();

  // Second loop (right curve of ∞)
  chassis.pid_odom_set({
    {{0_in, 0_in}, fwd, DRIVE_SPEED},
    {{12_in, -12_in}, fwd, DRIVE_SPEED},
    {{24_in, 0_in}, fwd, DRIVE_SPEED},
    {{12_in, 12_in}, fwd, DRIVE_SPEED}
  }, true);
}
 void moveToGPSPoint(double targetX, double targetY, double tolerance) {
    // Get current heading from GPS once at the beginning
    double desiredHeading = gps.get_heading();
    turnToFaceOppGPSPoint(targetX,targetY);


    while (true) {
        pros::gps_position_s_t gps_data = gps.get_position();
        double x = gps_data.x * 39.3701;  // meters to inches
        double y = gps_data.y * 39.3701;

        // Skip invalid data
        if (x == 0 && y == 0) {
            pros::delay(100);
            continue;
        }

        double dx = targetX - x;
        double dy = targetY - y;

        double distance = sqrt(dx * dx + dy * dy);
        if (distance <= tolerance) break;

        // Calculate direction to the point (we'll use this just for step)
        double targetAngle = atan2(dy, dx) * 180.0 / M_PI;
        if (targetAngle < 0) targetAngle += 360;

        // Move a small step toward the point while facing original heading
        double stepDistance = std::min(distance, 12.0); // move in small steps
        chassis.pid_drive_set(stepDistance * 1_in, DRIVE_SPEED, true);  // true = maintain heading
        chassis.pid_wait();

        // // Re-correct heading if drifted
        // double currentHeading = gps.get_heading();
        // double headingError = desiredHeading - currentHeading;
        // while (headingError > 180) headingError -= 360;
        // while (headingError < -180) headingError += 360;

        // if (fabs(headingError) > 3.0) {
        //     chassis.pid_turn_set(desiredHeading, TURN_SPEED);
        //     chassis.pid_wait();
        // }

        pros::delay(50);
    }

    // Stop the robot
    chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
    chassis.drive_set(0, 0);
}
void turnToFaceGPSPoint(double targetX, double targetY) {
    // Get current GPS position and heading 
    pros::gps_position_s_t position = gps.get_position();
    double robotX = position.x * 39.3701;  // meters to inches
    double robotY = position.y * 39.3701;
    double robotHeading = gps.get_heading();  // already in degrees (0-359)

    // Debug output for invalid position
    if (robotX == 0 && robotY == 0) {
        printf("[ERROR] GPS position invalid (0,0)\n");
        pros::lcd::print(0, "GPS invalid");
        return;
    }
    // Calculate desired global heading to face the point
    double dx = targetX - robotX;
    double dy = targetY - robotY;
    double targetHeading = atan2(dx, dy) * 180.0 / M_PI;
    if (targetHeading < 0) targetHeading += 360;

    // Calculate CW and CCW errors
    double cwError  = fmod((targetHeading - robotHeading + 360), 360); // CW turn (0–359)
    double ccwError = cwError - 360;                                   // CCW turn (-359–0)

    // Choose the shorter turn
    double turnAngle = (fabs(cwError) <= fabs(ccwError)) ? cwError : ccwError;

    // Debug prints
    printf("[DEBUG] Robot: (%.1f, %.1f) Heading: %.1f°\n", robotX, robotY, robotHeading);
    printf("[DEBUG] Target: (%.1f, %.1f) Target Heading: %.1f°\n", targetX, targetY, targetHeading);
    printf("[DEBUG] CW Error: %.1f°, CCW Error: %.1f°, Chosen Turn: %.1f°\n", cwError, ccwError, turnAngle);

    // Brain screen output
    pros::lcd::print(0, "Robot: X=%.1f Y=%.1f", robotX, robotY);
    pros::lcd::print(1, "Target: X=%.1f Y=%.1f", targetX, targetY);
    pros::lcd::print(2, "Head=%.1f Target=%.1f", robotHeading, targetHeading);
    pros::lcd::print(3, "CW=%.1f CCW=%.1f", cwError, ccwError);
    pros::lcd::print(4, "Turn=%.1f", turnAngle);
    chassis.drive_imu_reset();   

    // Perform turn
    chassis.pid_turn_set(turnAngle, TURN_SPEED);
    chassis.pid_wait();
    chassis.drive_imu_reset();   
    robotX = position.x * 39.3701;  // meters to inches
    robotY = position.y * 39.3701;
    robotHeading = gps.get_heading();  // already in degrees (0-359)

    //_______________Solution 2_________ with IMU
    //makes it tur to the same point each time, but the point is wrong
        // Get current GPS position
//   pros::gps_position_s_t position = gps.get_position();
// double robotX = position.x * 39.3701;
// double robotY = position.y * 39.3701;

// if (robotX == 0 && robotY == 0) {
//     printf("GPS position invalid\n");
//     return;
// }

// double dx = targetX - robotX;
// double dy = targetY - robotY;

// // Convert atan2 to VEX heading (0° = +Y, 90° = +X)
// double targetAngle = atan2(dy, dx) * 180.0 / M_PI;
// targetAngle = 90 - targetAngle;
// if (targetAngle < 0) targetAngle += 360;

// double currentIMU = imu.get_heading();  // 0-360°
// double turnAngle = targetAngle - currentIMU;
// while (turnAngle > 180) turnAngle -= 360;
// while (turnAngle <= -180) turnAngle += 360;

// printf("IMU heading: %.1f°, Target: %.1f°, Turn: %.1f°\n",
//        currentIMU, targetAngle, turnAngle);

// chassis.pid_turn_set(currentIMU + turnAngle, TURN_SPEED);
// chassis.pid_wait();
}




void turnToFaceOppGPSPoint(double targetX, double targetY) {
    // Get current GPS position and heading 
    pros::gps_position_s_t position = gps.get_position();
    double robotX = position.x * 39.3701;  // meters to inches
    double robotY = position.y * 39.3701;
    double robotHeading = gps.get_heading();  // already in degrees (0-359)

    // Debug output for invalid position
    if (robotX == 0 && robotY == 0) {
        printf("[ERROR] GPS position invalid (0,0)\n");
        pros::lcd::print(0, "GPS invalid");
        return;
    }
    // Calculate desired global heading to face the point
    double dx = targetX - robotX;
    double dy = targetY - robotY;
    double targetHeading =( atan2(dx, dy) * 180.0 / M_PI)+180;
    if (targetHeading < 0) targetHeading += 360;

    // Calculate CW and CCW errors
    double cwError  = fmod((targetHeading - robotHeading + 360), 360); // CW turn (0–359)
    double ccwError = cwError - 360;                                   // CCW turn (-359–0)

    // Choose the shorter turn
    double turnAngle = (fabs(cwError) <= fabs(ccwError)) ? cwError : ccwError;

    // Debug prints
    printf("[DEBUG] Robot: (%.1f, %.1f) Heading: %.1f°\n", robotX, robotY, robotHeading);
    printf("[DEBUG] Target: (%.1f, %.1f) Target Heading: %.1f°\n", targetX, targetY, targetHeading);
    printf("[DEBUG] CW Error: %.1f°, CCW Error: %.1f°, Chosen Turn: %.1f°\n", cwError, ccwError, turnAngle);

    // Brain screen output
    pros::lcd::print(0, "Robot: X=%.1f Y=%.1f", robotX, robotY);
    pros::lcd::print(1, "Target: X=%.1f Y=%.1f", targetX, targetY);
    pros::lcd::print(2, "Head=%.1f Target=%.1f", robotHeading, targetHeading);
    pros::lcd::print(3, "CW=%.1f CCW=%.1f", cwError, ccwError);
    pros::lcd::print(4, "Turn=%.1f", turnAngle);
    chassis.drive_imu_reset();   

    // Perform turn
    chassis.pid_turn_set(turnAngle , TURN_SPEED);
    chassis.pid_wait();
    chassis.drive_imu_reset();   
    robotX = position.x * 39.3701;  // meters to inches
    robotY = position.y * 39.3701;
    robotHeading = gps.get_heading();  // already in degrees (0-359)

}







void turnToAbsoluteHeading(double targetHeading) {
//     while (true) {
//         double currentHeading = gps.get_heading();  // in degrees

//         // Validate GPS heading (skip invalid data if needed)
//         if (std::isnan(currentHeading)) {
//             pros::delay(100);
//             continue;
//         }

//         // Calculate shortest angle difference [-180, 180]
//         double angleDiff = targetHeading - currentHeading;
//         while (angleDiff > 180) angleDiff -= 360;
//         while (angleDiff < -180) angleDiff += 360;

// while ((fabs(currentHeading - targetHeading) <= 2.0)) {
  
//         // Set PID turn to absolute target heading
        // chassis.pid_turn_set(targetHeading, TURN_SPEED);
//         chassis.pid_wait();

// }
        //  currentHeading = gps.get_heading();  // in degrees

// if (fabs(currentHeading - targetHeading) <= 2.0) {
//     break;
// }

        // // Set PID turn to absolute target heading
        // chassis.pid_turn_set(angleDiff, TURN_SPEED);
    //     // chassis.pid_wait();

    //     pros::delay(50);  // Small delay to avoid spamming
    // }
   // Get current heading from GPS
    double currentHeading = gps.get_heading();  // range: [0, 360)

    // Print debug
    std::cout << "[DEBUG] Current: " << currentHeading << " | Target: " << targetHeading << std::endl;

    // Calculate shortest turn angle [-180, 180]
    double turnAngle = targetHeading - currentHeading;
    if (turnAngle > 180) turnAngle -= 360;
    if (turnAngle < -180) turnAngle += 360;

    std::cout << "[DEBUG] Turning by: " << turnAngle << " degrees" << std::endl;

    // Execute PID turn by relative angle
    chassis.pid_turn_set(turnAngle, TURN_SPEED);
    chassis.pid_wait();

    // Optional: final heading
    double after = gps.get_heading();
    std::cout << "[DEBUG] After turn: " << after << std::endl;
}


// === GPS-BASED PID TURN ===
void gps_pid_turn(double targetAngle, double kP = 1.5, double kI = 0.0, double kD = 0.2, double tolerance = 1.0) {
    double integral = 0;
    double lastError = 0;
    double dt = 0.02;  // 20ms loop

    while (true) {
        double currentHeading = gps.get_heading();  // GPS heading 0–359

        // Normalize error to [-180, 180]
        double error = fmod((targetAngle - currentHeading + 540), 360) - 180;

        // Break if within tolerance
        if (fabs(error) < tolerance) break;

        // PID terms
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        double output = kP * error + kI * integral + kD * derivative;

        // Clamp output to safe motor range
        if (output > 100) output = 100;
        if (output < -100) output = -100;

        // Apply turn (tank style: left opposite of right)
        chassis.drive_set(output,-output);

        // Debug
        printf("[GPS PID] Target=%.1f, Current=%.1f, Error=%.1f, Out=%.1f\n", targetAngle, currentHeading, error, output);

        lastError = error;
        pros::delay(20);
    }

    // Stop motors after turn
        chassis.drive_set(0,0);
  
}