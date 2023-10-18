#include "raylib.h"
#include <math.h>
#include <stdio.h>



float* input_handler(Vector2 mousePosition, Vector2 robotPosition) {
    static float output[3]; // v1, v2, module_theta

    // Calculate module_theta to point towards the mouse
    float dx = mousePosition.x - robotPosition.x;
    float dy = mousePosition.y - robotPosition.y;
    output[2] = atan2f(dy, dx); // module_theta
    
    // Reset v1 and v2 to zero
    output[0] = 0.0f; // v1
    output[1] = 0.0f; // v2
    
    // Handle keyboard inputs for v1 and v2
    if (IsKeyDown(KEY_Q)) output[0] = 1.0f;  // v1 forward
    if (IsKeyDown(KEY_Z)) output[0] = -1.0f; // v1 backward
    if (IsKeyDown(KEY_E)) output[1] = 1.0f;  // v2 forward
    if (IsKeyDown(KEY_C)) output[1] = -1.0f; // v2 backward
    
    return output;
}

typedef struct Robot {
    float module_theta;
    float leftVelocity;
    float rightVelocity;
} Robot; 




int main() {
    InitWindow(800, 600, "Crab Drive Simulator");
    
    float v1 = 2.0f, v2 = 2.0f;
    float module_theta = 0.0f; // For demonstration, this will be dynamically updated later
    Vector2 position = { 400.0f, 300.0f };
    float theta_robot = 0.0f;
    float track_width = 50.0f;

    float dt = 1.0f / 60.0f;
    
    float bodyWidth = 60.0f;
    float bodyLength = 40.0f;
    float wheelWidth = 10.0f;
    float wheelLength = 20.0f;

    int counter = 0; 

    while (!WindowShouldClose()) {

        Vector2 mousePosition = GetMousePosition();
        float* inputs = input_handler(mousePosition, position); // assuming position is your robot's position

        // Update v1, v2, and module_theta based on control input (For now hardcoded for demonstration)
        float v1 = inputs[0];
        float v2 = inputs[1];
        float module_theta = inputs[2];
        
        float vx = (v1 + v2) * cosf(module_theta);
        float vy = (v1 + v2) * sinf(module_theta);
        
        float vx_global = vx * cosf(theta_robot) - vy * sinf(theta_robot);
        float vy_global = vx * sinf(theta_robot) + vy * cosf(theta_robot);
        
        position.x += vx_global * dt;
        position.y += vy_global * dt;

        float velocity_theta = ((v2 - v1) * cosf(module_theta)) / track_width;
        theta_robot += velocity_theta * dt;

        BeginDrawing();
        ClearBackground(RAYWHITE);

        Rectangle rec = { position.x, position.y, bodyWidth, bodyLength };
        DrawRectanglePro(rec, (Vector2){ bodyWidth / 2, bodyLength / 2 }, RAD2DEG * theta_robot - 90.0f, BLUE);

        float wheelOffset = track_width / 2.0f;
        float degreeOffset90 = 90.0f * DEG2RAD;
        // For Wheel 1
        Vector2 wheel1_local = { wheelOffset, 0.0f };
        Vector2 wheel1_global;
        wheel1_global.x = position.x + (wheel1_local.x * cosf(theta_robot + degreeOffset90) - wheel1_local.y * sinf(theta_robot + degreeOffset90));
        wheel1_global.y = position.y + (wheel1_local.x * sinf(theta_robot + degreeOffset90) + wheel1_local.y * cosf(theta_robot + degreeOffset90));

        // For Wheel 2
        Vector2 wheel2_local = { -wheelOffset, 0.0f };
        Vector2 wheel2_global;
        wheel2_global.x = position.x + (wheel2_local.x * cosf(theta_robot + degreeOffset90) - wheel2_local.y * sinf(theta_robot + degreeOffset90));
        wheel2_global.y = position.y + (wheel2_local.x * sinf(theta_robot + degreeOffset90) + wheel2_local.y * cosf(theta_robot + degreeOffset90));
        // Define and draw the rectangles representing the wheels
        Rectangle wheel1 = { wheel1_global.x, wheel1_global.y, wheelLength, wheelWidth };
        Rectangle wheel2 = { wheel2_global.x, wheel2_global.y, wheelLength, wheelWidth };

        DrawRectanglePro(wheel1, (Vector2){ wheelWidth / 2, wheelLength / 2 }, RAD2DEG * theta_robot + RAD2DEG * module_theta, GREEN);
        DrawRectanglePro(wheel2, (Vector2){ wheelWidth / 2, wheelLength / 2 }, RAD2DEG * theta_robot + RAD2DEG * module_theta, GREEN);

        counter++;

        if (counter % (60 * 30) == 0) {
            printf("Robot angle (Theta): %f radians\n", theta_robot);
            printf("Wheel 1 Global Position: x = %f, y = %f\n", wheel1.x, wheel1.y);
            printf("Wheel 2 Global Position: x = %f, y = %f\n", wheel2.x, wheel2.y);
            printf("Robot Center Position: x = %f, y = %f\n", position.x, position.y);
            printf("------\n");
        }
        EndDrawing();

    }
    
    CloseWindow();
    return 0;
}

