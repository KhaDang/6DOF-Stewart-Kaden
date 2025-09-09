#include "helpers.h"

// Helper function to map float values
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Rate limiting function to smooth transitions
float rateLimit(float target, float current) {
    float diff = target - current;
    float maxChange = 0.01; // Maximum change allowed per update
    
    if(diff > maxChange)
        return current + maxChange;
    else if(diff < -maxChange)
        return current - maxChange;
    else
        return target;
}

//function calculating needed servo rotation value
float getAlpha(int i,volatile float arr[]){
            
// Multipliers and offsets
    static const float DxMultiplier[6]   = { 1,  1,  1, -1, -1, -1};
    static const float AngleMultiplier[6]= { 1, -1,  1,  1, -1,  1};
    static const float OffsetAngle[6]    = { pi/6, pi/6, -pi/2, -pi/2, pi/6, pi/6 };
// Platform coordinates
    float platformPDx = DxMultiplier[i] * RD;
    float platformPDy = RD;
    float platformAngle = OffsetAngle[i] + AngleMultiplier[i] * radians(theta_r);

    float platformCoordsx = platformPDx * cos(platformAngle);
    float platformCoordsy = platformPDy * sin(platformAngle);
// Base coordinates
    float basePDx = DxMultiplier[i] * PD;
    float basePDy = PD;
    float baseAngle = OffsetAngle[i] + AngleMultiplier[i] * radians(theta_p);

    float baseCoordsx = basePDx * cos(baseAngle);
    float baseCoordsy = basePDy * sin(baseAngle);

    // Platform pivots
    float platformPivotx = platformCoordsx * cos(arr[3]) * cos(arr[5])
                         + platformCoordsy * (sin(arr[4]) * sin(arr[3]) * cos(arr[5]) - cos(arr[4]) * sin(arr[5]))
                         + arr[0];

    float platformPivoty = platformCoordsx * cos(arr[4]) * sin(arr[5])
                         + platformCoordsy * (cos(arr[3]) * cos(arr[5]) + sin(arr[3]) * sin(arr[4]) * sin(arr[5]))
                         + arr[1];

    float platformPivotz = -platformCoordsx * sin(arr[3])
                         + platformCoordsy * sin(arr[4]) * cos(arr[3])
                         + platformHeight + arr[2];
 // Delta values
    float deltaLx = baseCoordsx - platformPivotx;
    float deltaLy = baseCoordsy - platformPivoty;
    float deltaLz = -platformPivotz;

    float deltaL2Virtual = sqrt(deltaLx * deltaLx + deltaLy * deltaLy + deltaLz * deltaLz);

    // Servo arm math
    float l = pow(deltaL2Virtual, 2.0) - (pow(ConnectingArmLengthL2, 2.0) - pow(ServoArmLengthL1, 2.0));
    float m = 2 * ServoArmLengthL1 * platformPivotz;
    float n = 2 * ServoArmLengthL1 *
              (cos(theta_s[i] * pi / 180.0) * (platformPivotx - baseCoordsx) +
               sin(theta_s[i] * pi / 180.0) * (platformPivoty - baseCoordsy));

    // Safe angle calculation
    float denom = sqrt(m * m + n * n);
    float result = atan2(l, denom) - atan2(n, m);

    // if (isnan(result)) {
    //     Serial.printf("NaN at i=%d: l=%f, m=%f, n=%f, denom=%f\n",
    //                   i, l, m, n, denom);
    // }

    return result;
}