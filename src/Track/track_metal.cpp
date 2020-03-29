#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "AutoPinball/flip_flipper.h"

#include <iostream>
#include <stdlib.h> 

void draw_rotated_rectangle(cv::Mat& image, cv::RotatedRect rotatedRectangle){
    cv::Scalar color = cv::Scalar(0, 255.0, 0); // green

    // We take the edges that OpenCV calculated for us
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    cv::Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    // Now we can fill the rotated rectangle with our specified color
    cv::fillConvexPoly(image, vertices, 4, color);
}

// Returns 2 contour, hopefull, both flippers
std::vector<cv::RotatedRect> find_flippers(cv::Mat frame_in_question, double &time_of_last_flip_reset){
    std::cout << "FIND FLIPPERS" << std::endl;
    
    std::vector<std::vector<cv::Point> > cnts;
    int num_tries = 0;

    while(cnts.size() != 2 && num_tries < 5){
        cv::Mat mask;
        cv::GaussianBlur(frame_in_question, mask, cv::Size(15,15), 0);
        cv::cvtColor(mask, mask, cv::COLOR_BGR2HSV);

        cv::inRange(mask, cv::Scalar(20,120,59), cv::Scalar(40,220,190), mask);
        cv::erode(mask, mask, NULL, cv::Point(-1,-1), 3);
        cv::dilate(mask, mask, NULL, cv::Point(-1,-1), 3);

        cv::findContours(mask, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::cout << cnts.size() << std::endl;
        num_tries++;
    }

    time_of_last_flip_reset = ros::Time::now().toSec();

    if(num_tries >= 5){
        std::vector<cv::RotatedRect> empty;
        std::cout << "EMPTY" << std::endl;
        return empty;
    }
    else{
        std::cout << "NOT EMPTY" << std::endl;
        std::vector<cv::RotatedRect> rect_list;
        cv::RotatedRect left_rect = cv::minAreaRect(cnts[0]);
        cv::RotatedRect right_rect = cv::minAreaRect(cnts[1]);

        if(left_rect.center.x > right_rect.center.x){
            left_rect = cv::minAreaRect(cnts[1]);
            right_rect = cv::minAreaRect(cnts[0]);
        }

        rect_list.push_back(left_rect);
        rect_list.push_back(right_rect);
        return rect_list;
    }
}

std::vector<std::vector<cv::Point> > calculate_thresh(cv::Mat first_frame, cv::Mat current_frame, cv::Mat &frame_delta){
    cv::absdiff(first_frame, current_frame, frame_delta);
    cv::GaussianBlur(frame_delta, frame_delta, cv::Size(7,7), 0);
    cv::threshold(frame_delta, frame_delta, 25, 255, cv::THRESH_BINARY);
    cv::dilate(frame_delta, frame_delta, NULL, cv::Point(-1,-1), 2);
    std::vector<std::vector<cv::Point> > cnts;
    cv::findContours(frame_delta, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    return cnts;    
}

int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "Tracking_Ball");
    ros::NodeHandle nh;
    ros::Publisher publish_flipper = nh.advertise<AutoPinball::flip_flipper>("internal_flip_flipper", 10);


    // Camera object
    cv::VideoCapture camera(0);
    

    if(!camera.open(0)){
        std::cout << "Camera did not open properly" << std::endl;
        return 0;
    }

    // create frame objects
    cv::Mat raw;
    cv::Mat first_frame;
    cv::Mat img;
    cv::Mat frame_delta;
    cv::Mat raw_display;

    //get the first frame setup
    camera >> first_frame;
    sleep(1);
    camera >> first_frame;
    cv::cvtColor(first_frame, first_frame, CV_BGR2GRAY);

    // Contour map
    std::vector<std::vector<cv::Point> > my_contour;

    // Currnet coordinates for the ball
    int ball_x = -1;
    int ball_y = -1;

    // Threshholds for the size of the ball
    int THRESH_MAX = 1700;
    int THRESH_MIN = 1100;

    // Rectangle Contour
    std::vector<std::vector<cv::Point> > left_flip{{cv::Point(218,369), cv::Point(301,428), cv::Point(299,397), cv::Point(293,368), cv::Point(284,339), cv::Point(262,326), cv::Point(241,333), cv::Point(228,346)}};
    std::vector<std::vector<cv::Point> > right_flip{{cv::Point(421, 380), cv::Point(328, 427), cv::Point(332,393), cv::Point(351,360), cv::Point(378,345), cv::Point(406,354)}};

    // Track if we are currently flipping
    bool left_flipping = false;
    bool right_flipping = false;
    double left_last_flip_time = ros::Time::now().toSec();
    double right_last_flip_time = ros::Time::now().toSec();
    
    // Setup flipping messages
    float flip_delta = 0.2;
    AutoPinball::flip_flipper left_message;
    AutoPinball::flip_flipper right_message;
    left_message.time = flip_delta;
    right_message.time = flip_delta;
    left_message.flipper = 1;
    right_message.flipper = 2;

    // Set the initial ball center to off screen
    cv::Point ball_center(-1,-1);

    // Last time we reset the flipper position
    double time_of_last_flip_reset = ros::Time::now().toSec();
    std::vector<cv::RotatedRect> flipper_rects;

    while(ros::ok()){
        // Put new image into frame
        camera >> raw;

        // end of video stream
        if( raw.empty() ) break;

        // get a copy for displaying purposes
        raw_display = raw.clone();

        //Make it grayscale
        cv::cvtColor(raw, img, CV_BGR2GRAY);

        // Returns the list of contours seen
        my_contour = calculate_thresh(first_frame, img, frame_delta);

        // If that list is not empty...
        if (my_contour.size() > 0){
            // Draw the contours
            cv::drawContours(raw_display, my_contour, -1, cv::Scalar(100, 100, 100), 3);
            
            // For every contour in the list...
            for(int i=0; i < my_contour.size(); i++){
                // Find its area
                double area = cv::contourArea(my_contour[i]);
                
                // If that area is between our thresholds...
                if(area < THRESH_MAX && area > THRESH_MIN){
                    //Create a bounding rectangle around the current contour
                    cv::Rect pinball_rect = cv::Rect(cv::boundingRect(my_contour[i]));
                    ball_center.x = pinball_rect.x + pinball_rect.width/2;
                    ball_center.y = pinball_rect.y + pinball_rect.height/2;

                    // draw a circle around the ball
                    cv::circle(raw_display, ball_center, 20, cv::Scalar(0,0,0), 5);

                    // Test if the ball is inside the "flip zones" AND we are currently not flipping
                    if(cv::pointPolygonTest(left_flip[0], ball_center, false) >= 0 && !left_flipping){
                        left_flipping = true;
                        left_last_flip_time = ros::Time::now().toSec();
                        publish_flipper.publish(left_message);
                        std::cout << "LEFT!" << std::endl;
                    }
                    if(cv::pointPolygonTest(right_flip[0], ball_center, false) >= 0 && !right_flipping){
                        right_flipping = true;
                        right_last_flip_time = ros::Time::now().toSec();
                        publish_flipper.publish(right_message);
                        std::cout << "RIGHT!" << std::endl;
                    }
                }
                // The current contours area did not fit our thresholds
                else{
                    ball_center.x = -1;
                    ball_center.y = -1;
                }
            }
        }
        // There were no contours
        else{
            ball_center.x = -1;
            ball_center.y = -1;
        }

        // Draw the left flipper and right flipper boxes
        cv::drawContours(raw_display, left_flip, -1, cv::Scalar(255,255,0), 3);
        cv::drawContours(raw_display, right_flip, -1, cv::Scalar(255,255,0), 3);

        // Tell the flippers we are no longer flipping after a certain time
        if ((double)ros::Time::now().toSec() - left_last_flip_time > 2*flip_delta){
            left_flipping = false;
        }
        if ((double)ros::Time::now().toSec() - right_last_flip_time > 2*flip_delta){
            right_flipping = false;
        }

        if ((double)ros::Time::now().toSec() - time_of_last_flip_reset > 10){
            flipper_rects = find_flippers(raw, time_of_last_flip_reset);
        }
        /*
        Game plan for future Tyler:
        1. For each individual contour, calculate a minAreaRect
        2. Get the center and sort into lef/right flippers
        3. Write a function that can rotate a contour. 
        4. Rotate it to match the degree of rotation of the RotatedRect
        5. Translate it over the RotatedRect
        */

        for(int i = 0; i < flipper_rects.size(); i++){
            draw_rotated_rectangle(raw_display, flipper_rects[i]);
        }

        // Show video in new window
        cv::imshow("Frame", raw_display);

        // stop capturing by pressing q
        if( cv::waitKey(10) == 'q' ) break;  
    }

    return 0;
}