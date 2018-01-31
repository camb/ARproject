#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
using namespace cv;
#include <GL/glew.h> // include before gl.h and glfw.h
#include <GLFW/glfw3.h>
GLFWwindow* window;  // TODO: fix global var
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;
#include "shader.hpp"
using namespace std;


cv::Mat getWebcamStill() {
    // read in a single webcam frame
    VideoCapture webcam(0);
    // Set windows resolution
    webcam.set(CAP_PROP_FRAME_HEIGHT, 720);
    webcam.set(CAP_PROP_FRAME_WIDTH, 1280);
    // Set codec to YUYV to avoid MJPG corrupt JPEG warnings
    webcam.set(CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    Mat webcam_image;
    webcam.read(webcam_image);

    return webcam_image;
}


GLuint cvMatToGLuint(cv::Mat& input_mat) {
    GLuint textureID;
    cv::flip(input_mat, input_mat, 0);
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,            // Internal colour format to convert to
                 input_mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 input_mat.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 input_mat.ptr());        // The actual image data itself

    return textureID;
}


// TODO properly integrate these funcs
void cvtKeyPtoP(vector<KeyPoint>& kpts, vector<Point2f>& points) {
    points.clear();
    for (unsigned int i=0; i<kpts.size(); i++) points.push_back(kpts[i].pt);
}
void cvtPtoKpts(vector<KeyPoint>& kpts, vector<Point2f>& points) {
    kpts.clear();
    for (unsigned int i=0; i<points.size(); i++) kpts.push_back(KeyPoint(points[i],1));
}


// TODO resolve globals
double camD[] = {6.7649431228632795e+02, 0., 3.8262188058832749e+02, 0.,
    5.9941193806780484e+02, 1.6894241981264270e+02, 0., 0., 1.};
double distCoeffD[] = {5.5318827974857022e-02, -1.0129523116603711e+00,
    3.8895464611792836e-02, 2.5365684020675693e-02,
    2.6020235726385716e+00, 0., 0., 8.1013197871984710e-01};
Mat camera_matrix = Mat(3,3,CV_64FC1,camD);
Mat distortion_coefficients = Mat(5,1,CV_64FC1,distCoeffD);
Mat objPM;
vector<Point3d> objP;
vector<double> rv(3), tv(3);
Mat rvec(rv),tvec(tv); 
double _d[9] = {1,  0,  0,
    0,  -1, 0,
    0,  0,  -1}; //rotation: looking at -x axis
vector<KeyPoint> imgPointsOnPlane;
vector<uchar> status;
vector<float> track_error;
Mat rotM(3,3,CV_64FC1,_d);
double theta = 0.0,theta1 = 0.0,theta2 = 0.0,phi = 0.0,phi1 = 0.0,phi2 = 0.0,psi = 0.0,psi1 = 0.0,psi2 = 0.0;


void getPlanarSurface(vector<Point2f>& imgP) {    
    Rodrigues(rotM,rvec);

    // TODO moe this elsewhere
    objP.clear();
    objP.push_back(Point3d(0,0,0));
    objP.push_back(Point3d(5,0,0));
    objP.push_back(Point3d(5,5,0));
    objP.push_back(Point3d(0,5,0));
    Mat(objP).convertTo(objPM,CV_32F);

    solvePnP(objPM, Mat(imgP), camera_matrix, distortion_coefficients, rvec, tvec, true);

    Rodrigues(rvec,rotM);
}


int trackLatestImage(cv::Mat prev_webcam_image, cv::Mat cur_webcam_image) {
    // Detect 8x8 chessboard pattern, 7x7 internal points
    Size pattern = Size (7, 7);
    vector<Point2f> points1;
    vector<Point2f> points2;

    // Converting to black and white so points show up better later
    cvtColor(cur_webcam_image, cur_webcam_image, CV_BGR2GRAY);

    findChessboardCorners(prev_webcam_image, pattern, points1, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
    + CALIB_CB_FAST_CHECK);
    findChessboardCorners(cur_webcam_image, pattern, points2, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
    + CALIB_CB_FAST_CHECK);

    // Convert to color so red points show up
    cvtColor(cur_webcam_image, cur_webcam_image, CV_GRAY2BGR);

    //calc optical flow
    if (points1.size() == 0 || points2.size() == 0) {
        // we didn't track the points
        cout << "unable to track...\n";
        // TODO handle this without failing, use prev points possibly
    }
    else {
        // TODO cleanup this, take the 4 corner points from checkerboard
        vector<Point2f> tmp1;
        tmp1.push_back(points1[0]);
        tmp1.push_back(points1[6]);
        tmp1.push_back(points1[42]);
        tmp1.push_back(points1[48]);
        vector<Point2f> tmp2;
        tmp2.push_back(points2[0]);
        tmp2.push_back(points2[6]);
        tmp2.push_back(points2[42]);
        tmp2.push_back(points2[48]);
        points1 = tmp1;
        points2 = tmp2;


        calcOpticalFlowPyrLK(prev_webcam_image, cur_webcam_image, points1, points2, status, track_error, Size(30,30));

        cvtPtoKpts(imgPointsOnPlane, points2);

        //switch points vectors (next becomes previous)
        points1.clear();
        points1 = points2;
         
        //calculate camera pose
        getPlanarSurface(points1);

        for (auto p : points2) {
            Rect rect(p.x - 3, p.y - 3, 7, 7);
            rectangle(cur_webcam_image, rect, Scalar(0, 0, 255), 2, LINE_8, 0);
        }
    }

    // // Display Result
    imshow("cvWindow", cur_webcam_image);
    waitKey(30);
    return 0;
}


int setupGLFWWindow() {
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4); // aliasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow( 1280, 720, "GLFW Window", NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window.\n" );
        getchar();
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // background, possibly unnecessary
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);
    return 0;
}


int renderOpenGLFrame(GLuint& programID, GLuint& persp_programID,
                      GLuint& MatrixID, GLuint& persp_MatrixID,
                      cv::Mat& webcam_image, GLuint& TextureID,
                      GLuint& vertexbuffer, GLuint& uvbuffer,
                      GLuint& fgbuffer, GLuint& fgcolorbuffer,
                      glm::mat4& ortho_MVP, glm::mat4& persp_MVP) {
    // convert webcam_image for OpenGL texture
    GLuint Texture = cvMatToGLuint(webcam_image);

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use our shader
    glUseProgram(programID);

    // Send our transformation to the currently bound shader, 
    // in the "MVP" uniform
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &ortho_MVP[0][0]);

    // Bind our texture in Texture Unit 0
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, Texture);
    // Set our "myTextureSampler" sampler to use Texture Unit 0
    glUniform1i(TextureID, 0);

    // 1st attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(
        0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
        3,                  // size
        GL_FLOAT,           // type
        GL_FALSE,           // normalized?
        0,                  // stride
        (void*)0            // array buffer offset
    );
    // 2nd attribute buffer : UVs
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glVertexAttribPointer(
        1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
        2,                                // size : U+V => 2
        GL_FLOAT,                         // type
        GL_FALSE,                         // normalized?
        0,                                // stride
        (void*)0                          // array buffer offset
    );

    // Draw the BG triangles
    glDrawArrays(GL_TRIANGLES, 0, 2*3);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);


    // Draw the FG triangle
    // Clear the depth buffer
    glClear(GL_DEPTH_BUFFER_BIT);
    // Use a the FG shader
    glUseProgram(persp_programID);
    // Use the persp matrix
    glUniformMatrix4fv(persp_MatrixID, 1, GL_FALSE, &persp_MVP[0][0]);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, fgbuffer);
    glVertexAttribPointer(
        0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
        3,                  // size
        GL_FLOAT,           // type
        GL_FALSE,           // normalized?
        0,                  // stride
        (void*)0            // array buffer offset
    );
    // 2nd attribute buffer : colors
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, fgcolorbuffer);
    glVertexAttribPointer(
        1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
        3,                                // size
        GL_FLOAT,                         // type
        GL_FALSE,                         // normalized?
        0,                                // stride
        (void*)0                          // array buffer offset
    );

    // Draw FG tris
    glDrawArrays(GL_TRIANGLES, 0, 12*3);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    // Swap buffers
    glfwSwapBuffers(window);
    glfwPollEvents();
    return 0;
}


int main (int argc, char** argv) {
    setupGLFWWindow();

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders("shaders/SimpleVertexShader.vertexshader",
                                   "shaders/SimpleFragmentShader.fragmentshader");

    GLuint persp_programID = LoadShaders("shaders/SimpleVertexShader_persp.vertexshader",
                                         "shaders/SimpleFragmentShader_persp.fragmentshader");

    // Get a handle for our "MVP" uniform
    GLuint MatrixID = glGetUniformLocation(programID, "MVP");
    GLuint persp_MatrixID = glGetUniformLocation(persp_programID, "persp_MVP");
    // Old persp Projection: 45° Field of View, 4:3 ratio, dist: 0.1 to 100 units
    glm::mat4 persp_Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
    // ortho projection matrix for the background plate
    glm::mat4 ortho_Projection = glm::ortho(-1.0f, 1.0f,    // left,   right
                                            -1.0f, 1.0f,    // bottom, top
                                             0.1f, 100.0f);  // near,   far
    // Camera matrix
    glm::mat4 View  = glm::lookAt(
                        glm::vec3(0,0,5), // Camera is at (0,0,3), in World Space
                        glm::vec3(0,0,0), // and looks at the origin
                        glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
                      );
    // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 Model = glm::mat4(1.0f);
    // Our ModelViewProjection : multiplication of our 3 matrices
    glm::mat4 ortho_MVP  = ortho_Projection * View * Model; // Remember, matrix multiplication is the other way around
    glm::mat4 persp_MVP  = persp_Projection * View * Model; // Remember, matrix multiplication is the other way around
    
    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    // Background plate quad of 2 triangles
    static const GLfloat g_vertex_buffer_data[] = {
        -1.0f,   1.0f, 0.0f,
         1.0f,   1.0f, 0.0f,
        -1.0f,  -1.0f, 0.0f,
        // second triangle
        -1.0f,  -1.0f, 0.0f,
         1.0f,   1.0f, 0.0f,
         1.0f,  -1.0f, 0.0f,
    };
    // 2 UV coordinates for each vertex
    static const GLfloat g_uv_buffer_data[] = {
        0.0f, 1.0f,
        1.0f, 1.0f,
        0.0f, 0.0f,
        0.0f, 0.0f,
        1.0f, 1.0f,
        1.0f, 0.0f,
    };
    GLuint uvbuffer;
    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);
    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);


    // Foreground Cube
    // TODO make colors match corners
    static const GLfloat fg_vertex_buffer_data[] = {
        -1.0f,-1.0f,-1.0f, // triangle 1 : begin
        -1.0f,-1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f, // triangle 1 : end
        1.0f, 1.0f,-1.0f, // triangle 2 : begin
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f, // triangle 2 : end
        1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f,-1.0f,
        1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
        1.0f,-1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f,-1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f,
        1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f,
        1.0f,-1.0f, 1.0f
    };
    static const GLfloat fg_color_buffer_data[] = {
        0.583f,  0.771f,  0.014f,
        0.609f,  0.115f,  0.436f,
        0.327f,  0.483f,  0.844f,
        0.822f,  0.569f,  0.201f,
        0.435f,  0.602f,  0.223f,
        0.310f,  0.747f,  0.185f,
        0.597f,  0.770f,  0.761f,
        0.559f,  0.436f,  0.730f,
        0.359f,  0.583f,  0.152f,
        0.483f,  0.596f,  0.789f,
        0.559f,  0.861f,  0.639f,
        0.195f,  0.548f,  0.859f,
        0.014f,  0.184f,  0.576f,
        0.771f,  0.328f,  0.970f,
        0.406f,  0.615f,  0.116f,
        0.676f,  0.977f,  0.133f,
        0.971f,  0.572f,  0.833f,
        0.140f,  0.616f,  0.489f,
        0.997f,  0.513f,  0.064f,
        0.945f,  0.719f,  0.592f,
        0.543f,  0.021f,  0.978f,
        0.279f,  0.317f,  0.505f,
        0.167f,  0.620f,  0.077f,
        0.347f,  0.857f,  0.137f,
        0.055f,  0.953f,  0.042f,
        0.714f,  0.505f,  0.345f,
        0.783f,  0.290f,  0.734f,
        0.722f,  0.645f,  0.174f,
        0.302f,  0.455f,  0.848f,
        0.225f,  0.587f,  0.040f,
        0.517f,  0.713f,  0.338f,
        0.053f,  0.959f,  0.120f,
        0.393f,  0.621f,  0.362f,
        0.673f,  0.211f,  0.457f,
        0.820f,  0.883f,  0.371f,
        0.982f,  0.099f,  0.879f
    };
    GLuint fgvertexbuffer;
    glGenBuffers(1, &fgvertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, fgvertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(fg_vertex_buffer_data), fg_vertex_buffer_data, GL_STATIC_DRAW);
    GLuint fgcolorbuffer;
    glGenBuffers(1, &fgcolorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, fgcolorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(fg_color_buffer_data), fg_color_buffer_data, GL_STATIC_DRAW);

    
    cv::Mat cur_webcam_image = getWebcamStill();
    cv::Mat prev_webcam_image;
    GLuint Texture;
    namedWindow("cvWindow", WINDOW_NORMAL);
    do{
        // capture a new webcam image
        prev_webcam_image = cur_webcam_image;
        cur_webcam_image = getWebcamStill();

        // TODO multithread for speed?

        trackLatestImage(prev_webcam_image, cur_webcam_image);

        renderOpenGLFrame(programID, persp_programID,
                          MatrixID, persp_MatrixID,
                          prev_webcam_image, TextureID,
                          vertexbuffer, uvbuffer,
                          fgvertexbuffer, fgcolorbuffer,
                          ortho_MVP, persp_MVP);
    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );


    // Cleanup
    glDeleteBuffers(1, &uvbuffer);
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &fgvertexbuffer);
    glDeleteBuffers(1, &fgcolorbuffer);
    glDeleteProgram(programID);
    glDeleteTextures(1, &Texture);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();
    return 0;
}