#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

// Define the screen division macros and directions
#define SCN_UP screen_divide[0]
#define SCN_DOWN screen_divide[1]
#define SCN_LEFT screen_divide[2]
#define SCN_RIGHT screen_divide[3]

#define DIR_UP 10
#define DIR_DOWN 20
#define DIR_LEFT 30
#define DIR_RIGHT 40
#define DIR_NONE 50
#define DIR_UP_LEFT 60
#define DIR_UP_RIGHT 70
#define DIR_DOWN_LEFT 80
#define DIR_DOWN_RIGHT 90

#define Shoot_Box_Width 100
#define Shoot_Box_Height 100

/////Variable
// Real Goal box
bool Goal_det_flg = false;

Rect Goal_Box;

vector<Point> goal_trace;
int Goal_trace_direction = DIR_NONE;

// virtual Goal box (for shooting)
Rect Shoot_Box;

vector<vector<Point>> screen_divide = vector<vector<Point>>(4);

// Define HSV color range variables
int lowerH = 94;
int upperH = 128;
int lowerS = 64;
int upperS = 153;
int lowerV = 37;
int upperV = 255;

// Function to calculate the center of a rectangle
Point calculateCenter(Rect rect)
{
    return Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
}

// Function to extract color and find the center
Point extractColorAndFindCenter(Mat &inputFrame, Mat &binaryImage)
{
    // Convert Goal_Box to HSV color space
    Mat hsvImage;
    cvtColor(inputFrame, hsvImage, COLOR_BGR2HSV);

    // Define the lower and upper bounds for HSV color range
    Scalar lowerBound(lowerH, lowerS, lowerV);
    Scalar upperBound(upperH, upperS, upperV);

    // Create a mask to extract the color within the defined range
    Mat mask;
    inRange(hsvImage, lowerBound, upperBound, mask);

    // Find contours in the mask
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Initialize the center point
    Point center(0, 0);

    // Find the largest contour
    double maxArea = 0;
    int maxAreaIdx = -1;
    for (int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxAreaIdx = i;
        }
    }

    // If a contour is found, calculate the center of the contour
    if (maxAreaIdx != -1)
    {
        Moments moments_ = moments(contours[maxAreaIdx]);
        if (moments_.m00 != 0)
        {
            center.x = int(moments_.m10 / moments_.m00);
            center.y = int(moments_.m01 / moments_.m00);
        }
    }

    // Draw the contour and center on the input frame (for visualization)
    drawContours(inputFrame, contours, maxAreaIdx, Scalar(0, 0, 255), 2); // Red contour
    circle(inputFrame, center, 5, Scalar(0, 0, 255), -1);                 // Red center point

    // Copy the mask to the binaryImage
    mask.copyTo(binaryImage);

    return center;
}

// Function to calculate relative position and direction
int calculateRelativePositionAndDirection(const Point &goalCenter, const Point &shootCenter, int &traceDirection)
{
    Point relativePosition = goalCenter - shootCenter;

    if (relativePosition.y < -Shoot_Box_Width / 2)
    {
        traceDirection = DIR_UP;
        if (relativePosition.x < -Shoot_Box_Width / 2)
        {
            traceDirection = DIR_UP_LEFT;
        }
        else if (relativePosition.x > Shoot_Box_Width / 2)
        {
            traceDirection = DIR_UP_RIGHT;
        }
    }
    else if (relativePosition.y > Shoot_Box_Width / 2)
    {
        traceDirection = DIR_DOWN;
        if (relativePosition.x < -Shoot_Box_Width / 2)
        {
            traceDirection = DIR_DOWN_LEFT;
        }
        else if (relativePosition.x > Shoot_Box_Width / 2)
        {
            traceDirection = DIR_DOWN_RIGHT;
        }
    }
    else if (relativePosition.x < -Shoot_Box_Width / 2)
    {
        traceDirection = DIR_LEFT;
        if (relativePosition.y < -Shoot_Box_Width / 2)
        {
            traceDirection = DIR_UP_LEFT;
        }
        else if (relativePosition.y > Shoot_Box_Width / 2)
        {
            traceDirection = DIR_DOWN_LEFT;
        }
    }
    else if (relativePosition.x > Shoot_Box_Width / 2)
    {
        traceDirection = DIR_RIGHT;
        if (relativePosition.y < -Shoot_Box_Width / 2)
        {
            traceDirection = DIR_UP_RIGHT;
        }
        else if (relativePosition.y > Shoot_Box_Width / 2)
        {
            traceDirection = DIR_DOWN_RIGHT;
        }
    }
    else
    {
        traceDirection = DIR_NONE;
    }

    return traceDirection;
}

int8_t DrawObj(Mat &image)
{
    // Extract color and find center
    Mat binaryImage;
    Point extractedCenter = extractColorAndFindCenter(image, binaryImage);

    // Draw the Shoot_Box as a fixed rectangle (e.g., blue)
    rectangle(image, Shoot_Box, Scalar(0, 0, 255), 2);

    // Draw the Goal_Box (e.g., green) if Goal_det_flg is true
    if (Goal_det_flg)
    {
        rectangle(image, Goal_Box, Scalar(0, 255, 0), 2);

        // Calculate the relative position of Goal_Box with respect to Shoot_Box
        Point goalCenter = calculateCenter(Goal_Box);
        goalCenter = extractedCenter;
        Point shootCenter = calculateCenter(Shoot_Box);
        int traceDirection;
        calculateRelativePositionAndDirection(goalCenter, shootCenter, traceDirection);

        // Output relative position and angle
        cout << "Relative Position: (" << goalCenter.x - shootCenter.x << ", " << goalCenter.y - shootCenter.y << ")" << endl;

        // Display Goal_trace_direction information
        string directionText;

        switch (traceDirection)
        {
        case DIR_NONE:
            directionText = "DIR NONE";
            break;
        case DIR_UP:
            directionText = "DIR UP";
            break;
        case DIR_DOWN:
            directionText = "DIR DOWN";
            break;
        case DIR_LEFT:
            directionText = "DIR LEFT";
            break;
        case DIR_RIGHT:
            directionText = "DIR RIGHT";
            break;
        case DIR_UP_LEFT:
            directionText = "DIR UP LEFT";
            break;
        case DIR_UP_RIGHT:
            directionText = "DIR UP RIGHT";
            break;
        case DIR_DOWN_LEFT:
            directionText = "DIR DOWN LEFT";
            break;
        case DIR_DOWN_RIGHT:
            directionText = "DIR DOWN RIGHT";
            break;
        default:
            directionText = "Unknown Direction";
        }

        putText(image, directionText, Point(5, 60), 2, 0.8, CV_RGB(0, 255, 0));
        string traceText = format("%4.2f %4.2f", goalCenter.x - shootCenter.x, goalCenter.y - shootCenter.y);
        putText(image, traceText, Point(5, 40), 2, 0.8, CV_RGB(0, 255, 0));
    }
    else
    {
        Goal_trace_direction = DIR_NONE;
    }

    // Output the extracted center
    cout << "Center of the extracted color: (" << extractedCenter.x << ", " << extractedCenter.y << ")" << endl;

    // Display the binary image
    imshow("Binary Image", binaryImage);

    return Goal_trace_direction;
}

// Callback function for HSV trackbars
void onTrackbar(int, void *)
{
    // You can update the color extraction parameters here if needed
}

int main()
{
    VideoCapture camera(0);

    if (!camera.isOpened())
    {
        cerr << "Error: 카메라를 열 수 없습니다." << endl;
        return -1;
    }

    // Set the camera properties
    camera.set(CAP_PROP_FRAME_WIDTH, 320);
    camera.set(CAP_PROP_FRAME_HEIGHT, 240);
    camera.set(CAP_PROP_FPS, 30);

    Goal_det_flg = true;

    namedWindow("Drawn Image", WINDOW_NORMAL);
    namedWindow("Control", WINDOW_NORMAL);
    namedWindow("Binary Image", WINDOW_NORMAL); // New window for binary image

    // Create trackbars for HSV color range adjustment
    createTrackbar("LowerH", "Control", &lowerH, 180, onTrackbar);
    createTrackbar("UpperH", "Control", &upperH, 180, onTrackbar);
    createTrackbar("LowerS", "Control", &lowerS, 255, onTrackbar);
    createTrackbar("UpperS", "Control", &upperS, 255, onTrackbar);
    createTrackbar("LowerV", "Control", &lowerV, 255, onTrackbar);
    createTrackbar("UpperV", "Control", &upperV, 255, onTrackbar);

    while (true)
    {
        Mat frame;
        camera >> frame;

        if (frame.empty())
        {
            cerr << "NO FRAME" << endl;
            break;
        }

        // Initialize Shoot_Box and Goal_Box
        Shoot_Box = Rect(100, 100, Shoot_Box_Width, Shoot_Box_Height); // Example Shoot_Box

        // Call the DrawObj() function to draw and extract color
        DrawObj(frame);
        std::cout << "Goal_trace_direction : " << Goal_trace_direction << std::endl;

        // Display the image
        imshow("Drawn Image", frame);

        // 'q' 키를 누르면 루프를 종료합니다.
        if (waitKey(1) == 'q')
        {
            break;
        }
    }

    camera.release();
    destroyAllWindows();

    return 0;
}



