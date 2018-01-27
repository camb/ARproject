#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <thread>

// TODO: commented out while troubleshooting OpenGL
#include <opencv2/opencv.hpp>
using namespace cv;

#include <GL/glew.h> // include before gl.h and glfw.h

#include <GLFW/glfw3.h>
GLFWwindow* window;  // TODO: fix global var

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;


#include "shader.hpp"

// #include <GL/gl.h>
// #include <GL/glut.h>
using namespace std;


int runOpenGL() {
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
        getchar();
        glfwTerminate();
        return -1;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders("shaders/SimpleVertexShader.vertexshader",
                                   "shaders/SimpleFragmentShader.fragmentshader");


    static const GLfloat g_vertex_buffer_data[] = { 
        -1.0f, -1.0f, 0.0f,
         1.0f, -1.0f, 0.0f,
         0.0f,  1.0f, 0.0f,
    };

    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    do{

        // Clear the screen
        glClear( GL_COLOR_BUFFER_BIT );

        // Use our shader
        glUseProgram(programID);

        // 1rst attribute buffer : vertices
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

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, 3); // 3 indices starting at 0 -> 1 triangle

        glDisableVertexAttribArray(0);

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );

    // Cleanup VBO
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteProgram(programID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();
    return 0;
}


int runOpenCV() {
        //////////////////////////////////////
    // OpenCV Video Input Code beings here
    //////////////////////////////////////
    VideoCapture webcam(0);
    if (!webcam.isOpened()){
        CV_Assert("webcam failed to open");
    }
    // Set windows resolution
    webcam.set(CAP_PROP_FRAME_HEIGHT, 720);
    webcam.set(CAP_PROP_FRAME_WIDTH, 1280);
    // Set codec to YUYV to avoid MJPG corrupt JPEG warnings
    webcam.set(CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));

    while (true) {
        // Capture video frame
        Mat webcamFrame;
        webcam.read(webcamFrame);

        // Detect 8x8 chessboard pattern, 7x7 internal points
        Size pattern = Size (7, 7); 
        vector<Point2f> corners;
        findChessboardCorners(webcamFrame, pattern, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        + CALIB_CB_FAST_CHECK);

        Scalar color = Scalar(0, 0, 255); // pure red
        for (auto p : corners) {
            Rect rect(p.x - 2, p.y - 2, 5, 5);
            rectangle(webcamFrame, rect, color, 2, LINE_8, 0);
        }

        // Display Result
        imshow("Camera Output", webcamFrame);
        if (waitKey(30) >= 0) break; // break on keystroke
    }
    return 0;
}


int main (int argc, char** argv) {
    cout << "Foo\n";
    //runOpenGL();
    thread first (runOpenGL);
    thread second (runOpenCV);
    first.join();
    second.join();
    cout << "Bar\n";

    return 0;
}