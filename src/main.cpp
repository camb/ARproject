#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>  // for printing glm matices
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h> // needed for distortion tutorial
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "shader.hpp"


cv::Mat getWebcamStill() {
    cv::VideoCapture webcam(0);
    webcam.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    webcam.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // Set codec to YUYV to avoid MJPG corrupt JPEG warnings
    webcam.set(cv::CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));
    cv::Mat webcam_image;
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
                 input_mat.cols,    // Image width  i.e. 640 for Kinect in standard mode
                 input_mat.rows,    // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 GL_BGR,            // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 input_mat.ptr());  // The actual image data itself

    return textureID;
}


// TODO resolve globals
GLFWwindow* window;  // TODO: fix global var

double camD[] = {6.7649431228632795e+02, 0., 3.8262188058832749e+02, 0.,
     5.9941193806780484e+02, 1.6894241981264270e+02, 0., 0., 1.};
double distCoeffD[] = {5.5318827974857022e-02, -1.0129523116603711e+00,
    3.8895464611792836e-02, 2.5365684020675693e-02,
    2.6020235726385716e+00, 0., 0., 8.1013197871984710e-01};
cv::Mat camera_matrix = cv::Mat(3,3,CV_64FC1,camD);
cv::Mat dist_coeff = cv::Mat(5,1,CV_64FC1,distCoeffD);
cv::Mat obj_pts_mtrx;
std::vector<cv::Point3d> obj_pts;
std::vector<double> rv(3), tv(3);
cv::Mat rvec(rv),tvec(tv);
double _d[9] = {1,   0,   0,
                0,  -1,   0,
                0,   0,  -1}; //rotation: looking at -x axis
std::vector<uchar> status;
std::vector<float> track_error;
cv::Mat rotM(3,3,CV_64FC1,_d);
glm::mat4 GLMView;


void solveViewMatrix(std::vector<cv::Point2f>& corners) {
    // TODO fix calibrating camera
    std::cout << "precalib camera matrix:\n" << camera_matrix << std::endl << std::endl;
    // TODO fix crashing and resolve intrinsic camera properties: camera matrix & distortion
    cv::calibrateCamera(obj_pts, corners, cv::Size(1280, 720), camera_matrix, dist_coeff, rvec, tvec, 0);
    std::cout << "postcalib camera matrix:\n" << camera_matrix << std::endl << std::endl;
    return;

    cv::Rodrigues(rotM, rvec);

    cv::solvePnP(obj_pts_mtrx, cv::Mat(corners), camera_matrix, dist_coeff, rvec, tvec, false);

    cv::Rodrigues(rvec, rotM);

    rotM = rotM.t();
    tvec = -rotM * tvec;

    // setup 4x4 CV formatted View matrix in the object frame
    cv::Mat cvView(4, 4, rotM.type()); // cvView is 4x4
    cvView( cv::Range(0,3), cv::Range(0,3) ) = rotM * 1; // copies R into cvView
    cvView( cv::Range(0,3), cv::Range(3,4) ) = tvec * 1; // copies tvec into cvView
    // fill the last row of cvView (NOTE: depending on your types, use float or double)
    double *p = cvView.ptr<double>(3);
    p[0] = p[1] = p[2] = 0; p[3] = 1;

    // converting the OpenCV View matrix to glm matrix?
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            GLMView[j][i] = cvView.at<double>(i, j);
        }
    }

    // OpenCV are stored in row-major order to OpenGL ones, in column-major order
    // GLMView = glm::transpose(GLMView);

    // cvView is your 4x4 matrix in the OpenCV frame
    glm::rotate(GLMView, 180.0f, glm::vec3(1.0, 0.0, 0.0));
    glm::rotate(GLMView, 180.0f, glm::vec3(0.0, 0.0, 1.0));


    // TODO Debugging => Camera matrix
    GLMView  = glm::lookAt(glm::vec3(0,5,0), // Camera is at (0,0,3), in World Space
                           glm::vec3(0,0,0), // and looks at the origin
                           glm::vec3(0,0,-1)  // Head is up (set to 0,-1,0 to look upside-down)
                          );
    // std::cout << "GLM Matrix:\n:" << to_string(GLMView) << std::endl << std::endl;
}


int solveCameraView(cv::Mat prev_img, cv::Mat next_img) {
    static std::vector<cv::Point2f> next_corners;
    // static std::vector<cv::Point2f> prev_corners;
    std::vector<cv::Point2f> next_pts;
    // std::vector<cv::Point2f prev_pts;
    cv::cvtColor(next_img, next_img, CV_BGR2GRAY);

    bool found_next = cv::findChessboardCorners(next_img, cv::Size(7, 7), next_pts, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                      						    + cv::CALIB_CB_FAST_CHECK);
    if (found_next) {
        next_corners = {next_pts[0], next_pts[6], next_pts[42], next_pts[48]};
        // Convert to color so red points show up
        cv::cvtColor(next_img, next_img, CV_GRAY2BGR);
        // Draw corners in CV GLFW window
        for (auto p : next_corners)
            cv::rectangle(next_img, cv::Rect(p.x-3, p.y-3, 7, 7), cv::Scalar(0, 0, 255), 2, cv::LINE_8, 0);

        solveViewMatrix(next_corners);
    }
    // Display image
    cv::imshow("cvWindow", next_img);
    cv::waitKey(30);
    return 0;
    { // contained code is only needed for calc optical flow func
        // Track chessboards as needed
        // bool found_prev = false;
        // if (prev_corners.empty() && next_corners.empty()) {
        //     found_prev = findChessboardCorners(prev_img, cv::Size(7, 7), prev_pts, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        //                           + CALIB_CB_FAST_CHECK);
        //     if (found_prev)
        //         prev_corners = {prev_pts[0], prev_pts[6], prev_pts[42], prev_pts[48]};
        // }
        // else if (!next_corners.empty()) {
        //     prev_corners = next_corners;
        // }
        // bool found_next = findChessboardCorners(next_img, cv::Size(7, 7), next_pts, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        //                   + CALIB_CB_FAST_CHECK);
        // if (found_next)
        //     next_corners = {next_pts[0], next_pts[6], next_pts[42], next_pts[48]};

        // // Convert to color so red points show up
        // cvtColor(next_img, next_img, CV_GRAY2BGR);

        // if (prev_corners.empty() || next_corners.empty()) {
        //     // TODO improve handling if it didn't track the points
        //     std::cout << "unable to track...\n";
        // }
        // else {
        //     // TODO Integrate calculate optical flow if needed
        //     // calcOpticalFlowPyrLK(prev_img, next_img,
        //     //                      prev_corners, next_corners,
        //     //                      status, track_error, cv::Size(30,30));

        //     // Switch points vectors (next becomes previous)
        //     prev_corners = next_corners;
        //     next_corners.clear();

        //     // Draw corners in CV GLFW window
        //     for (auto p : prev_corners)
        //         rectangle(next_img, Rect(p.x-3, p.y-3, 7, 7), Scalar(0, 0, 255), 2, LINE_8, 0);
             
        //     // Calculate camera View matrix
        //     solveViewMatrix(prev_corners);

        // // // Display Image
        // imshow("cvWindow", next_img);
        // waitKey(30);
        // return 0;
    }
}


int setupGLFWWindow() {
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf(stderr, "Failed to initialize GLFW\n" );
        getchar();
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4); // aliasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(1280, 720, "GLFW Window", NULL, NULL);
    if( window == NULL ){
        fprintf(stderr, "Failed to open GLFW window.\n");
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

    // Set background color to black
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);
    return 0;
}


int render3D(GLuint& ProgramID, GLuint& PerspProgramID,
                      GLuint& OrthoMatrixID, GLuint& PerspMatrixID,
                      cv::Mat& webcam_image, GLuint& TextureID,
                      GLuint& vertexbuffer, GLuint& uvbuffer,
                      GLuint& fgbuffer, GLuint& fgcolorbuffer,
                      glm::mat4& OrthoMVP, glm::mat4& PerspMVP)
{
    { // Draw BG plate
        // convert webcam_image for OpenGL texture
        GLuint Texture = cvMatToGLuint(webcam_image);
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use shader
        glUseProgram(ProgramID);

        // Send transformation to the currently bound shader, 
        // in the "MVP" uniform
        glUniformMatrix4fv(OrthoMatrixID, 1, GL_FALSE, &OrthoMVP[0][0]);

        // Bind texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        // Set "myTextureSampler" sampler to use Texture Unit 0
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
    }

    // Draw the FG triangle
    // Clear the depth buffer
    glClear(GL_DEPTH_BUFFER_BIT);
    // Use a the FG shader
    glUseProgram(PerspProgramID);
    // Use the persp matrix
    glUniformMatrix4fv(PerspMatrixID, 1, GL_FALSE, &PerspMVP[0][0]);

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
    GLuint ProgramID = LoadShaders("shaders/BGPlate.vertexshader",
                                   "shaders/BGPlate.fragmentshader");
    GLuint PerspProgramID = LoadShaders("shaders/Cube.vertexshader",
                                        "shaders/Cube.fragmentshader");

    // Get a handle for our "MVP" uniform
    GLuint OrthoMatrixID = glGetUniformLocation(ProgramID, "MVP");
    GLuint PerspMatrixID = glGetUniformLocation(PerspProgramID, "PerspMVP");
    // Persp Projection: 30° Field of View, 16:9 ratio, dist: 0.1 to 1000 units
    glm::mat4 PerspProj = glm::perspective(glm::radians(30.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
    // std::cout << "PerspProj: \n" << to_string(PerspProj) << std::endl << std::endl;
    // Ortho projection matrix for the background plate
    glm::mat4 OrthoProj = glm::ortho(-1.0f, 1.0f,    // left,   right
                                     -1.0f, 1.0f,    // bottom, top
                                      0.1f, 100.0f); // near,   far
    // Camera matrix
    glm::mat4 View  = glm::lookAt(glm::vec3(0,0,5), // Camera is at (0,0,3), in World Space
                                  glm::vec3(0,0,0), // and looks at the origin
                                  glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
                                 );
    // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 Model = glm::mat4(1.0f);
    // Our ModelViewProjection : multiplication of our 3 matrices
    glm::mat4 OrthoMVP  = OrthoProj * View * Model; // Remember, matrix multiplication is the other way around
    glm::mat4 PerspMVP  = PerspProj * View * Model; // Remember, matrix multiplication is the other way around
    
    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureID  = glGetUniformLocation(ProgramID, "myTextureSampler");

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
    static const GLfloat fg_vertex_buffer_data[] = {
        -1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f,
         1.0f, 1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f,
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

    // Setup object pattern
    obj_pts.clear();
    obj_pts.push_back(cv::Point3d(0,0,0));
    obj_pts.push_back(cv::Point3d(5,0,0));
    obj_pts.push_back(cv::Point3d(5,5,0));
    obj_pts.push_back(cv::Point3d(0,5,0));
    cv::Mat(obj_pts).convertTo(obj_pts_mtrx, CV_32F);


    cv::Mat prev_img, next_img = getWebcamStill();
    GLuint Texture;
    cv::namedWindow("cvWindow", cv::WINDOW_NORMAL);
    do{
        // capture a new webcam image
        prev_img = next_img;
        next_img = getWebcamStill();

        solveCameraView(prev_img, next_img);

        // apply the solved View for the persp 3D render
        // PerspMVP  = PerspProj * GLMView * Model;

        // render the prev_image and FG 3D object
        render3D(ProgramID, PerspProgramID,
                 OrthoMatrixID, PerspMatrixID,
                 prev_img, TextureID,
                 vertexbuffer, uvbuffer,
                 fgvertexbuffer, fgcolorbuffer,
                 OrthoMVP, PerspMVP);
    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );


    // Cleanup gl assets
    glDeleteBuffers(1, &uvbuffer);
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &fgvertexbuffer);
    glDeleteBuffers(1, &fgcolorbuffer);
    glDeleteProgram(ProgramID);
    glDeleteTextures(1, &Texture);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}