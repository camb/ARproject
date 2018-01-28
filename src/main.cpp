#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <thread>

// OpenCV includes
#include <opencv2/opencv.hpp>
using namespace cv;

// OpenGL includes
#include <GL/glew.h> // include before gl.h and glfw.h
#include <GLFW/glfw3.h>
GLFWwindow* window;  // TODO: fix global var
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "shader.hpp"
using namespace std;

// TODO remove?
// #include <GL/gl.h>
// #include <GL/glut.h>


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


int trackLatestImage(cv::Mat& webcam_image) {
    // Detect 8x8 chessboard pattern, 7x7 internal points
    Size pattern = Size (7, 7); 
    vector<Point2f> corners;
    findChessboardCorners(webcam_image, pattern, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
    + CALIB_CB_FAST_CHECK);

    Scalar color = Scalar(0, 0, 255); // pure red
    for (auto p : corners) {
        Rect rect(p.x - 2, p.y - 2, 5, 5);
        rectangle(webcam_image, rect, color, 2, LINE_8, 0);
    }

    // Display Result
    //imshow("Camera Output", webcam_image);
    //if (waitKey(30) >= 0) break; // break on keystroke

    // TODO Change return type from int to solved transform matrix for OpenGL
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

    // TODO reenable these when adding 3D
    // Enable depth test
    //glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    //glDepthFunc(GL_LESS);
    return 0;
}


int renderOpenGLFrame(GLuint& programID, GLuint& MatrixID,
                       cv::Mat& webcam_image, GLuint& TextureID,
                       GLuint& vertexbuffer, GLuint& uvbuffer,
                       glm::mat4& MVP) {

    // convert webcam_image for OpenGL texture
    GLuint Texture = cvMatToGLuint(webcam_image);

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use our shader
    glUseProgram(programID);

    // Send our transformation to the currently bound shader, 
    // in the "MVP" uniform
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

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

    // Draw the triangles
    glDrawArrays(GL_TRIANGLES, 0, 2*3);

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

    // Get a handle for our "MVP" uniform
    GLuint MatrixID = glGetUniformLocation(programID, "MVP");

    // Old persp Projection: 45° Field of View, 4:3 ratio, dist: 0.1 to 100 units
    //glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);

    // ortho projection matrix for the background plate
    glm::mat4 Projection = glm::ortho(-1.0f, 1.0f, -1.0f, 1.0f, 0.1f, 100.0f);

    // Camera matrix
    glm::mat4 View  = glm::lookAt(
                        glm::vec3(0,0,3), // Camera is at (0,0,3), in World Space
                        glm::vec3(0,0,0), // and looks at the origin
                        glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
                      );
    // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 Model = glm::mat4(1.0f);
    // Our ModelViewProjection : multiplication of our 3 matrices
    glm::mat4 MVP  = Projection * View * Model; // Remember, matrix multiplication is the other way around
    
    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    // 2 triangles that makeup the background plate
    static const GLfloat g_vertex_buffer_data[] = {
        -1.0f, 1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        -1.0f, -1.0f, 0.0f,
        // second triangle
        -1.0f, -1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
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


    
    cv::Mat cur_webcam_image = getWebcamStill();
    cv::Mat prev_webcam_image;
    GLuint Texture;
    do{
        prev_webcam_image = cur_webcam_image;
        // capture a new webcam image
        cur_webcam_image = getWebcamStill();

        // thread 1 tracks newest webcam image and outputs transforms for OpenGL
        //thread first (trackLatestImage, ref(cur_webcam_image));

        // thread 2 renders previous webcam image with overlayed 3D
        // TODO fix this thread, it's seg faulting
        // thread second (renderOpenGLFrame,
        //                ref(programID), ref(MatrixID),
        //                ref(prev_webcam_image), ref(TextureID),
        //                ref(vertexbuffer), ref(uvbuffer),
        //                ref(MVP));
        renderOpenGLFrame(programID, MatrixID,
                          prev_webcam_image, TextureID,
                          vertexbuffer, uvbuffer,
                          MVP);

        // synchronize threads, display prev image, repeat with newest image
        // second.join();
        // first.join();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );


    // Cleanup
    glDeleteBuffers(1, &uvbuffer);
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteProgram(programID);
    glDeleteTextures(1, &Texture);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();
    return 0;
}