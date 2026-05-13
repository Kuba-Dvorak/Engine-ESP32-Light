#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <math.h>
#include <regex>
#include <algorithm>
#include <array>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "Arduino.h"
#include "driver/periph_ctrl.h"
#include "soc/lcd_cam_struct.h"
#include "soc/lcd_cam_reg.h"
#include "driver/gpio.h"      // gpio_matrix_out
#include "rom/lldesc.h"       // lldesc_t
#include "esp_heap_caps.h"    // heap_caps_malloc pro DMA paměť
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_private/gdma.h"
#include "esp_private/esp_clk.h"
#include "soc/rtc.h"
#include "soc/esp32s3/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "regi2c_ctrl.h"
#include "esp32s3/rom/cache.h"


//fully operational 3D raycasting engine

float getRandomfloat(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    float returnableValue = float((dis(gen)*(max-min+1))+min);
    return returnableValue;
}

int getRandomInt(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    int returnableValue = int((dis(gen)*(max-min+1))+min);
    return returnableValue;
}


struct simple3D_Pos_float {
    float x, y, z;

    simple3D_Pos_float(float x = 0, float y = 0, float z = 0) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    simple3D_Pos_float changedBy(float changerX, float changerY, float changerZ) {
        return simple3D_Pos_float(x + changerX, y + changerY, z + changerZ);
    }
};


class Vector3D_float {
public:
    float absoluteLenght;
    simple3D_Pos_float myPos;

    Vector3D_float(simple3D_Pos_float impPos) {
        this->myPos = impPos;
        this->absoluteLenght = std::sqrt((myPos.x * myPos.x) + (myPos.y * myPos.y) + (myPos.z * myPos.z));
    }

    simple3D_Pos_float getVec() {
        return myPos;
    }

    simple3D_Pos_float& getVecRef() {
        return myPos;
    }

    float dotProduct(Vector3D_float &secondVec) {
        return (secondVec.getVecRef().x * myPos.x) + (secondVec.getVecRef().y * myPos.y) + (secondVec.getVecRef().z * myPos.z);
    }

    float dotProductSimple(simple3D_Pos_float &secondVec) {
        return (secondVec.x * myPos.x) + (secondVec.y * myPos.y) + (secondVec.z * myPos.z);
    }

    void setVector(simple3D_Pos_float impPos) {
        myPos.x = impPos.x;
        myPos.y = impPos.y;
        myPos.z = impPos.z;
        absoluteLenght = std::sqrt((myPos.x * myPos.x) + (myPos.y * myPos.y) + (myPos.z * myPos.z));
    }

    float absoluteValue() {
        return absoluteLenght;
    }

    float crossProduct2D(Vector3D_float &secondVec) {
        return  myPos.x * secondVec.getVecRef().y - myPos.y * secondVec.getVecRef().x;
    }

    Vector3D_float crossProduct3D(Vector3D_float &secondVec) {
        return Vector3D_float(simple3D_Pos_float((myPos.y * secondVec.myPos.z) - (myPos.z * secondVec.myPos.y), (myPos.z * secondVec.myPos.x) - (myPos.x * secondVec.myPos.z), (myPos.x * secondVec.myPos.y) - (myPos.y * secondVec.myPos.x)));
    }

    float getDeterminant(Vector3D_float &normal, Vector3D_float &headingVec) {
        return normal.dotProduct(headingVec);
    }
};


struct SimpleColor {
    int red, blue, green;
    static constexpr float koeficient = (1.0f / 64);
    SimpleColor(int red = 0, int green = 0, int blue = 0) {
        this->red = red;
        this->blue = blue;
        this->green = green;
    }
    uint8_t convertToBinary() {
        uint8_t finalColor = 0b11000000;
        finalColor |= int((red * koeficient));
        finalColor |= int((green * koeficient)) << 2;
        finalColor |= int((blue * koeficient)) << 4;
        return finalColor;
    }
};


struct screenAndCameraInfo {
    float numberAmpX, numberAmpY;
    int screenHeight, screenWidth;

    screenAndCameraInfo(float numberAmpX = 0, float numberAmpY = 0, int screenHeight = 0, int screenWidth = 0) {
        this->numberAmpX = numberAmpX;
        this->numberAmpY = numberAmpY;
        this->screenHeight = screenHeight;
        this->screenWidth = screenWidth;
    }
};


class Position3D_float {
public:

    simple3D_Pos_float myPos;

    Position3D_float(simple3D_Pos_float impPos = simple3D_Pos_float(0,0,0)) {
        this->myPos.x = impPos.x;
        this->myPos.y = impPos.y;
        this->myPos.z = impPos.z;
    }

    simple3D_Pos_float getPos() {
        return myPos;
    }


    void setPosition(simple3D_Pos_float impPos) {
        myPos.x = impPos.x;
        myPos.y = impPos.y;
        myPos.z = impPos.z;
    }

    void changePosition(simple3D_Pos_float impPos) {
        myPos.x += impPos.x;
        myPos.y += impPos.y;
        myPos.z += impPos.z;
    }

    Vector3D_float makeAVector(Position3D_float &secondPos) {
        return Vector3D_float(simple3D_Pos_float(secondPos.myPos.x - myPos.x,secondPos.myPos.y - myPos.y,secondPos.myPos.z - myPos.z));
    }

    Vector3D_float makeAUnitVector(Position3D_float secondPos) {
        float distance = std::sqrt(std::pow(myPos.x - secondPos.myPos.x, 2) + std::pow(myPos.y - secondPos.myPos.y, 2) + std::pow(myPos.z - secondPos.myPos.z, 2));

        if (distance < 0.05) {
            distance = 0.06;
        }

        float nowDistance = 1.0f / distance;

        return Vector3D_float(simple3D_Pos_float((secondPos.myPos.x - myPos.x) * nowDistance,(secondPos.myPos.y - myPos.y) * nowDistance,(secondPos.myPos.z - myPos.z) * nowDistance));
    }

    Vector3D_float makeA2DVector(Position3D_float &secondPos) {
        return Vector3D_float(simple3D_Pos_float(myPos.x - secondPos.myPos.x,myPos.y - secondPos.myPos.y,0));
    }

    float absoluteDistance(Position3D_float &secondPos) {
        return std::sqrt(std::pow(secondPos.myPos.x - myPos.x,2) + std::pow(secondPos.myPos.y - myPos.y,2) + std::pow(secondPos.myPos.z - myPos.z,2));
    }

    float absoluteDistanceSimple(simple3D_Pos_float &secondPos) {
        return std::sqrt(std::pow(secondPos.x - myPos.x,2) + std::pow(secondPos.y - myPos.y,2) + std::pow(secondPos.z - myPos.z,2));
    }

    Position3D_float changedBy(simple3D_Pos_float changePos) {
        return Position3D_float(simple3D_Pos_float(myPos.x + changePos.x, myPos.y + changePos.y, myPos.z + changePos.z));
    }

    Position3D_float makeIntoScreensCord(screenAndCameraInfo const &impInfo, float quickZ) {

        if (myPos.z > 0.01) {
            return Position3D_float(simple3D_Pos_float(std::round(((myPos.x * quickZ) * impInfo.numberAmpX) + (impInfo.screenWidth * 0.5f)), std::round(((myPos.y * quickZ) * impInfo.numberAmpY) + (impInfo.screenHeight * 0.5f)), myPos.z));
        }

        if (std::abs(myPos.z) < 0.005) {
            myPos.z = 0.006;
        }

        return Position3D_float(simple3D_Pos_float(std::round(((myPos.x * 100) * impInfo.numberAmpX) + (impInfo.screenWidth * 0.5f)), std::round(((myPos.y * 100) * impInfo.numberAmpY) + (impInfo.screenHeight * 0.5f)), myPos.z));
    }

    Position3D_float makeIntoGradiantable(screenAndCameraInfo const &impInfo, float quickZ) {
        if (std::abs(myPos.z) < 0.005) {
            myPos.z = 0.006;
        }
        return Position3D_float(simple3D_Pos_float(std::round(((myPos.x * quickZ) * impInfo.numberAmpX) + (impInfo.screenWidth * 0.5f)), std::round(((myPos.y * quickZ) * impInfo.numberAmpY) + (impInfo.screenHeight * 0.5f)), quickZ));
    }
};


std::array<float, 2> getGradiantsfloat(Position3D_float &pointA, Position3D_float &pointB, Position3D_float &pointC) {
    float diffX1 = pointB.myPos.x - pointA.myPos.x;
    float diffX2 = pointC.myPos.x - pointB.myPos.x;
    float diffY1 = pointB.myPos.y - pointA.myPos.y;
    float diffY2 = pointC.myPos.y - pointB.myPos.y;
    float diffZ1 = pointB.myPos.z - pointA.myPos.z;
    float diffZ2 = pointC.myPos.z - pointB.myPos.z;
    float determinant = (diffX1 * diffY2 - diffX2 * diffY1);
    if (std::abs(determinant) <= 10) {
        return {0,0};
    }
    determinant = 1.0f / determinant;
    float gradiantX = (diffY2 * diffZ1 - diffZ2 * diffY1) * (determinant);
    float gradiantY = (diffX1 * diffZ2 - diffZ1 * diffX2) * (determinant);
    return {gradiantX, gradiantY};
}


class Point_float {
public:
    Position3D_float position = {};
    simple3D_Pos_float xVector;
    simple3D_Pos_float zVector;
    simple3D_Pos_float yVector;

    Point_float(simple3D_Pos_float impPosition = simple3D_Pos_float(0,0,0)) {
        this->position.myPos = impPosition;
        this->xVector = {0,0,0};
        this->zVector = {0,0,0};
        this->yVector = {0,0,0};
    }

    void setupUnitVectors(simple3D_Pos_float referencePoint) {
        if (position.myPos.x >= referencePoint.x) {
            xVector = {1,0,0};
        }
        if (position.myPos.x < referencePoint.x) {
            xVector = {-1,0,0};
        }

        if (position.myPos.y >= referencePoint.y) {
            yVector = {0,1,0};
        }
        if (position.myPos.y < referencePoint.y) {
            yVector = {0,-1,0};
        }

        if (position.myPos.z >= referencePoint.z) {
            zVector = {0,0,1};
        }
        if (position.myPos.z < referencePoint.z) {
            zVector = {0,0,-1};
        }
    }

    Position3D_float getPos() {
        return position;
    }

    Position3D_float& getPosRef() {
        return position;
    }

    void setPos(simple3D_Pos_float changePos) {
        position.myPos = changePos;
    }

    void changePos(simple3D_Pos_float changePos) {
        position.myPos.x += changePos.x;
        position.myPos.y += changePos.y;
        position.myPos.z += changePos.z;
    }

    Position3D_float getRelativePos(Vector3D_float &headingVec, Vector3D_float &upVector, Vector3D_float &rightVector, Position3D_float &headingOrigin) {

        Vector3D_float vectorOS = headingOrigin.makeAVector(position);

        return Position3D_float(simple3D_Pos_float(vectorOS.dotProduct(rightVector), vectorOS.dotProduct(upVector), vectorOS.dotProduct(headingVec)));
    }
};


struct LightRay_float {
    Position3D_float myPos;
    float intenzity;
    float lenght;
    SimpleColor color;
    float highestLight;
    Vector3D_float headingVec = Vector3D_float(simple3D_Pos_float());
    int maxIndexCur = -1;

    LightRay_float(Position3D_float impPos = simple3D_Pos_float(0,0,0), float intenzity = 0, SimpleColor myCol = SimpleColor(255,255,255), Vector3D_float definingVec = Vector3D_float(simple3D_Pos_float(0,0,0)),
        float lenghtConst = 1, float maxDistance = 1) {
        this->myPos = impPos;
        this->intenzity = intenzity;
        this->color = myCol;
        this->highestLight = maxDistance;
        this->headingVec = definingVec;
        this->lenght = lenghtConst;
        this->maxIndexCur = -1;
    }
};


enum class LightTypes {
    pointLike = 1,
    paralel = 2
};


class GlobalPolygon_float {
private:
    std::array<Point_float, 3> definingGlobalPoints = {Point_float(simple3D_Pos_float()), Point_float(simple3D_Pos_float()), Point_float(simple3D_Pos_float())};
    std::array<Position3D_float, 3> definingLocalPoints = {Position3D_float(), Position3D_float(), Position3D_float()};
    std::array<Position3D_float, 3> definingGradiantablePoints = {Position3D_float(), Position3D_float(), Position3D_float()};
    SimpleColor originalColor;
    SimpleColor outlineColor;
    SimpleColor dislayedColor;
    bool blockification;
    float blockLOD;
    bool shouldDraw;
    std::array<Vector3D_float, 2> lightingVectors = {Vector3D_float(simple3D_Pos_float()), Vector3D_float(simple3D_Pos_float())};
    Vector3D_float mainNormal = Vector3D_float(simple3D_Pos_float());
    int id;

    std::array<int, 4> localMinsAmaxs;
    std::array<float, 2> localGradiant;
    uint8_t localConvertedColor;
    uint8_t localOutlineColor;
    bool localxMajority = false;
    float localBlockification = 1;
    std::array<float, 3> localKoeficients;
    std::array<int, 3> localIndexesNumbers;

    void prepresentAssets(screenAndCameraInfo &cameraInfo) {
        localMinsAmaxs = minsAndMaxs(cameraInfo);
        localGradiant = getGradiantsfloat(definingGradiantablePoints[0], definingGradiantablePoints[1], definingGradiantablePoints[2]);

        localConvertedColor = dislayedColor.convertToBinary();
        localOutlineColor = outlineColor.convertToBinary();

        localxMajority = compareMinsAndMaxs(localMinsAmaxs);

        localIndexesNumbers = minsAndMaxsAMiddle(localxMajority);

        localKoeficients = getKoeficients(localxMajority, localIndexesNumbers);

        localBlockification = 1;

        if (blockification) {
            float sizer = (localMinsAmaxs[0] - localMinsAmaxs[2]) * (localMinsAmaxs[1] - localMinsAmaxs[3]) * 0.5;
            float middleZ = (definingLocalPoints[0].myPos.z + definingLocalPoints[1].myPos.z + definingLocalPoints[2].myPos.z) * 0.334;
            localBlockification = int(blockLOD * sqrt(sizer) / middleZ);
        }

        if (localBlockification <= 1) {
            localBlockification = 1;
        }

        if (localBlockification >= 20) {
            localBlockification = 20;
        }
    }

    std::array<int, 4> minsAndMaxs(screenAndCameraInfo const &camerasInfo) {
        std::array<int, 4> minsAMax = {int(definingLocalPoints[0].myPos.x), int(definingLocalPoints[0].myPos.y), int(definingLocalPoints[0].myPos.x), int(definingLocalPoints[0].myPos.y)};
        for (int i = 0; i < 3; i += 1) {
            if (definingLocalPoints[i].myPos.x >= minsAMax[0]) {
                minsAMax[0] = int(definingLocalPoints[i].myPos.x);
            }
            if (definingLocalPoints[i].myPos.y >= minsAMax[1]) {
                minsAMax[1] = int(definingLocalPoints[i].myPos.y);
            }

            if (definingLocalPoints[i].myPos.x <= minsAMax[2]) {
                minsAMax[2] = int(definingLocalPoints[i].myPos.x);
            }
            if (definingLocalPoints[i].myPos.y <= minsAMax[3]) {
                minsAMax[3] = int(definingLocalPoints[i].myPos.y);
            }
        }

        //border pripady

        if (minsAMax[2] <= 0) {
            minsAMax[2] = 0;
        }
        if (minsAMax[3] <= 0) {
            minsAMax[3] = 0;
        }
        if (minsAMax[0] >= camerasInfo.screenWidth) {
            minsAMax[0] = camerasInfo.screenWidth-2;
        }
        if (minsAMax[1] >= camerasInfo.screenHeight) {
            minsAMax[1] = camerasInfo.screenHeight-2;
        }
        return minsAMax;
    }

    std::array<int, 3> minsAndMaxsAMiddle(bool &xMajority) {
        std::array<int, 3> minsAMax = {0,0,0};

        if (xMajority) {
            for (int i = 0; i < 3; i += 1) {
                if (definingLocalPoints[i].myPos.y >= definingLocalPoints[minsAMax[0]].myPos.y) {
                    minsAMax[0] = i;
                }
                if (definingLocalPoints[i].myPos.y <= definingLocalPoints[minsAMax[1]].myPos.y) {
                    minsAMax[1] = i;
                }
            }
            minsAMax[2] = (3 - minsAMax[0] - minsAMax[1]);

            if (minsAMax[2] > 2 || minsAMax[2] < 0) {
                minsAMax[2] = 2;
            }
        }


        else {
            for (int i = 0; i < 3; i += 1) {
                if (definingLocalPoints[i].myPos.x >= definingLocalPoints[minsAMax[0]].myPos.x) {
                    minsAMax[0] = i;
                }
                if (definingLocalPoints[i].myPos.x <= definingLocalPoints[minsAMax[1]].myPos.x) {
                    minsAMax[1] = i;
                }
            }
            minsAMax[2] = (3 - minsAMax[0] - minsAMax[1]);

            if (minsAMax[2] > 2 || minsAMax[2] < 0) {
                minsAMax[2] = 2;
            }
        }
        return minsAMax;
    }

    bool compareMinsAndMaxs(std::array<int, 4> const &infoArray) {
        if (std::abs(infoArray[0] - infoArray[2]) >= std::abs(infoArray[3] - infoArray[1])) {
            return true;
        }
        return false;
    }

    std::array<float, 3> getKoeficients(bool xMajority, std::array<int, 3> const &infos) {
        std::array<float, 3> returningKoeficients = {};

        float diveder0y = (definingLocalPoints[infos[0]].myPos.y - definingLocalPoints[infos[1]].myPos.y);
        float diveder1y = (definingLocalPoints[infos[2]].myPos.y - definingLocalPoints[infos[1]].myPos.y);
        float diveder2y = (definingLocalPoints[infos[0]].myPos.y - definingLocalPoints[infos[2]].myPos.y);

        if (diveder0y <= 0.01 && diveder0y >= 0) {
            diveder0y = 0.02;
        }

        if (diveder1y <= 0.01 && diveder1y >= 0) {
            diveder1y = 0.02;
        }

        if (diveder2y <= 0.01 && diveder2y >= 0) {
            diveder2y = 0.02;
        }

        if (diveder0y >= -0.01 && diveder0y <= 0) {
            diveder0y = -0.02;
        }

        if (diveder1y >= -0.01 && diveder1y <= 0) {
            diveder1y = -0.02;
        }

        if (diveder2y >= -0.01 && diveder2y <= 0) {
            diveder2y = -0.02;
        }

        float diveder0x = (definingLocalPoints[infos[0]].myPos.x - definingLocalPoints[infos[1]].myPos.x);
        float diveder1x = (definingLocalPoints[infos[2]].myPos.x - definingLocalPoints[infos[1]].myPos.x);
        float diveder2x = (definingLocalPoints[infos[0]].myPos.x - definingLocalPoints[infos[2]].myPos.x);

        if (diveder0x <= 0.01 && diveder0x >= 0) {
            diveder0x = 0.02;
        }

        if (diveder1x <= 0.01 && diveder1x >= 0) {
            diveder1x = 0.02;
        }

        if (diveder2x <= 0.01 && diveder2x >= 0) {
            diveder2x = 0.02;
        }

        if (diveder0x >= -0.01 && diveder0x <= 0) {
            diveder0x = -0.02;
        }

        if (diveder1x >= -0.01 && diveder1x <= 0) {
            diveder1x = -0.02;
        }

        if (diveder2x >= -0.01 && diveder2x <= 0) {
            diveder2x = -0.02;
        }

        if (xMajority) {

            returningKoeficients[0] = diveder0x / diveder0y;
            returningKoeficients[1] = diveder1x / diveder1y;
            returningKoeficients[2] = diveder2x / diveder2y;
        }

        else {

            returningKoeficients[0] = diveder0y / diveder0x;
            returningKoeficients[1] = diveder1y / diveder1x;
            returningKoeficients[2] = diveder2y / diveder2x;
        }
        return returningKoeficients;
    }

    void changeColor(SimpleColor &newColor) {
        if (newColor.red >= 255) {
            newColor.red = 255;
        }
        if (newColor.red <= 0) {
            newColor.red = 0;
        }

        if (newColor.green >= 255) {
            newColor.green = 255;
        }
        if (newColor.green <= 0) {
            newColor.green = 0;
        }

        if (newColor.blue >= 255) {
            newColor.blue = 255;
        }
        if (newColor.blue <= 0) {
            newColor.blue = 0;
        }
        dislayedColor = newColor;
    }

public:
    bool reactToLight;

    GlobalPolygon_float(std::array<Point_float, 3> definingGlobalPoints, std::array<Position3D_float, 3> definingLocalPoints, std::array<Position3D_float, 3> definingGradiantablePoints, SimpleColor impCol, int index, bool drawOut, bool reactToLight) {
        this->definingGlobalPoints = definingGlobalPoints;
        this->definingLocalPoints = definingLocalPoints;
        this->definingGradiantablePoints = definingGradiantablePoints;
        this->originalColor = impCol;
        this->lightingVectors[0] = definingGlobalPoints[0].getPosRef().makeAVector(definingGlobalPoints[1].getPosRef());
        this->lightingVectors[1] = definingGlobalPoints[0].getPosRef().makeAVector(definingGlobalPoints[2].getPosRef());
        this->mainNormal = lightingVectors[0].crossProduct3D(lightingVectors[1]);
        this->dislayedColor = impCol;
        this->id = index;
        this->shouldDraw = drawOut;
        this->reactToLight = reactToLight;
    }

    void drawOutPolygonSDL2SuperFast(uint8_t* zBufferImp, uint8_t* colorsBuffer, screenAndCameraInfo const &cameraInfo, bool outLine, int outlineThickness) {
        if (localxMajority) {
            bool leftRight = false;
            int minX = 0;
            int maxX = 0;
            if (definingLocalPoints[localIndexesNumbers[0]].myPos.x + (localKoeficients[0] * (localMinsAmaxs[3] - definingLocalPoints[localIndexesNumbers[0]].myPos.y)) > definingLocalPoints[localIndexesNumbers[0]].myPos.x + (localKoeficients[2] * (localMinsAmaxs[3] - definingLocalPoints[localIndexesNumbers[0]].myPos.y))) {
                leftRight = true;
            }

            for (int yPos = localMinsAmaxs[3]; yPos < localMinsAmaxs[1]+1; yPos += localBlockification) {
                int xPos2 = int(definingLocalPoints[localIndexesNumbers[0]].myPos.x + (localKoeficients[0] * (yPos - definingLocalPoints[localIndexesNumbers[0]].myPos.y)));
                int xPos1 = 0;
                if (yPos >= definingLocalPoints[localIndexesNumbers[2]].myPos.y) {
                    xPos1 = int(definingLocalPoints[localIndexesNumbers[0]].myPos.x + (localKoeficients[2] * (yPos - definingLocalPoints[localIndexesNumbers[0]].myPos.y)));
                }

                else {
                    xPos1 = int(definingLocalPoints[localIndexesNumbers[2]].myPos.x + (localKoeficients[1] * (yPos - definingLocalPoints[localIndexesNumbers[2]].myPos.y)));
                }

                if (leftRight) {
                    minX = xPos1;
                    maxX = xPos2;
                }
                else {
                    minX = xPos2;
                    maxX = xPos1;
                }

                if (minX <= 0) {
                    minX = 0;
                }

                if (maxX >= cameraInfo.screenWidth) {
                    maxX = cameraInfo.screenWidth-1;
                }

                if (minX >= cameraInfo.screenWidth) {
                    minX = cameraInfo.screenWidth-1;
                }

                if (maxX <= 0) {
                    maxX = 0;
                }

                for (int xPos = minX; xPos < maxX+1; xPos += 1) {
                    uint8_t globalZ = uint8_t(1.0f / (definingGradiantablePoints[0].myPos.z + (xPos - definingGradiantablePoints[0].myPos.x) * localGradiant[0] + (yPos - definingGradiantablePoints[0].myPos.y) * localGradiant[1]));

                    if (blockification) {
                        for (int yPosReal = yPos; yPosReal < yPos + localBlockification; yPosReal += 1) {
                            if (yPosReal >= cameraInfo.screenHeight) {
                                yPosReal += localBlockification;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] && globalZ > 0) {
                                if (outLine && ((xPos - (outlineThickness / globalZ) < minX) || (xPos + (outlineThickness / globalZ) > maxX))) {
                                    colorsBuffer[xPos + (yPosReal * cameraInfo.screenWidth)] = localOutlineColor;
                                }
                                else {
                                    colorsBuffer[xPos + (yPosReal * cameraInfo.screenWidth)] = localConvertedColor;
                                }
                                zBufferImp[xPos + (yPosReal * cameraInfo.screenWidth)] = globalZ;
                            }
                        }
                    }

                    else {

                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                            if (outLine && ((xPos - (outlineThickness / globalZ) < minX) || (xPos + (outlineThickness / globalZ) > maxX))) {
                                colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = localOutlineColor;
                            }
                            else {
                                colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = localConvertedColor;
                            }
                            zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] = globalZ;
                        }
                    }
                }
            }
        }

        else {
            bool leftRight = false;
            int minY = 0;
            int maxY = 0;

            if (definingLocalPoints[localIndexesNumbers[0]].myPos.y + (localKoeficients[2] * (localMinsAmaxs[2] - definingLocalPoints[localIndexesNumbers[0]].myPos.x)) < definingLocalPoints[localIndexesNumbers[0]].myPos.y + (localKoeficients[0] * (localMinsAmaxs[2] - definingLocalPoints[localIndexesNumbers[0]].myPos.x))) {
                leftRight = true;
            }

            for (int xPos = localMinsAmaxs[2]; xPos < localMinsAmaxs[0]+1; xPos += localBlockification) {

                int yPos2 = std::round(definingLocalPoints[localIndexesNumbers[0]].myPos.y + (localKoeficients[0] * (xPos - definingLocalPoints[localIndexesNumbers[0]].myPos.x)));
                int yPos1 = 0;

                if (xPos >= definingLocalPoints[localIndexesNumbers[2]].myPos.x) {
                    yPos1 = std::round(definingLocalPoints[localIndexesNumbers[0]].myPos.y + (localKoeficients[2] * (xPos - definingLocalPoints[localIndexesNumbers[0]].myPos.x)));
                }

                else {
                    yPos1 = std::round(definingLocalPoints[localIndexesNumbers[2]].myPos.y + (localKoeficients[1] * (xPos - definingLocalPoints[localIndexesNumbers[2]].myPos.x)));
                }

                if (leftRight) {
                    minY = yPos1;
                    maxY = yPos2;
                }
                else {
                    minY = yPos2;
                    maxY = yPos1;
                }

                if (minY <= 0) {
                    minY = 0;
                }

                if (maxY >= cameraInfo.screenHeight) {
                    maxY = cameraInfo.screenHeight-1;
                }

                if (minY >= cameraInfo.screenHeight) {
                    minY = cameraInfo.screenHeight-1;
                }

                if (maxY <= 0) {
                    maxY = 0;
                }

                for (int yPos = minY; yPos < maxY+1; yPos += 1) {
                    uint8_t globalZ = uint8_t(1.0f / (definingGradiantablePoints[0].myPos.z + (xPos - definingGradiantablePoints[0].myPos.x) * localGradiant[0] + (yPos - definingGradiantablePoints[0].myPos.y) * localGradiant[1]));

                    if (blockification) {
                        for (int xPosReal = xPos; xPosReal < xPos + localBlockification; xPosReal += 1) {
                            if (xPosReal >= cameraInfo.screenWidth) {
                                xPosReal += localBlockification;
                                continue;
                            }

                            if (globalZ < zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                                if (outLine && ((yPos - (outlineThickness / globalZ) < minY) || (yPos + (outlineThickness / globalZ) > maxY))) {
                                    colorsBuffer[xPosReal + (yPos * cameraInfo.screenWidth)] = localOutlineColor;
                                }
                                else {
                                    colorsBuffer[xPosReal + (yPos * cameraInfo.screenWidth)] = localConvertedColor;
                                }
                                zBufferImp[xPosReal + (yPos * cameraInfo.screenWidth)] = globalZ;
                            }
                        }
                    }
                    else {
                        if (globalZ < zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] && globalZ > 0) {
                            if (outLine && ((yPos - (outlineThickness / globalZ) < minY) || (yPos + (outlineThickness / globalZ) > maxY))) {
                                colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = localOutlineColor;
                            }
                            else {
                                colorsBuffer[xPos + (yPos * cameraInfo.screenWidth)] = localConvertedColor;
                            }
                            zBufferImp[xPos + (yPos * cameraInfo.screenWidth)] = globalZ;
                        }
                    }
                }
            }
        }
    }

    void thisChange(std::array<Point_float, 3> newdefiningGlobalPoints, std::array<Position3D_float, 3> newdefiningLocalPoints, std::array<Position3D_float, 3> newdefiningGradiantablePoints, bool drawOut, screenAndCameraInfo &cameraInfo) {
        definingGlobalPoints = newdefiningGlobalPoints;
        definingLocalPoints = newdefiningLocalPoints;
        definingGradiantablePoints = newdefiningGradiantablePoints;
        lightingVectors[0] = definingGlobalPoints[0].getPosRef().makeAVector(definingGlobalPoints[1].getPosRef());
        lightingVectors[1] = definingGlobalPoints[0].getPosRef().makeAVector(definingGlobalPoints[2].getPosRef());
        mainNormal = lightingVectors[0].crossProduct3D(lightingVectors[1]);
        shouldDraw = drawOut;
        prepresentAssets(cameraInfo);
    }

    void playerChange(std::array<Position3D_float, 3> newdefiningLocalPoints, std::array<Position3D_float, 3> newdefiningGradiantablePoints, screenAndCameraInfo &cameraInfo, bool blockification, float blockDetail, bool drawOut) {
        definingLocalPoints = newdefiningLocalPoints;
        definingGradiantablePoints = newdefiningGradiantablePoints;
        shouldDraw = drawOut;
        prepresentAssets(cameraInfo);
    }

    void changeOriginalColor(SimpleColor newColor) {
        if (newColor.red >= 255) {
            newColor.red = 255;
        }
        if (newColor.red <= 0) {
            newColor.red = 0;
        }

        if (newColor.green >= 255) {
            newColor.green = 255;
        }
        if (newColor.green <= 0) {
            newColor.green = 0;
        }

        if (newColor.blue >= 255) {
            newColor.blue = 255;
        }
        if (newColor.blue <= 0) {
            newColor.blue = 0;
        }
        originalColor = newColor;
    }

    void rollBackColor() {
        dislayedColor = originalColor;
    }

    void changeLightColor(LightRay_float &light, float furtheness) {
        float newIntenzity = light.intenzity / (furtheness * light.lenght);
        SimpleColor newColor;
        newColor.red = int(dislayedColor.red + std::round(newIntenzity * light.color.red));
        newColor.green = int(dislayedColor.green + std::round(newIntenzity * light.color.green));
        newColor.blue = int(dislayedColor.blue + std::round(newIntenzity * light.color.blue));
        changeColor(newColor);
    }

    void lightingDuty(LightRay_float &lightPoint, int curentPos) {
        Vector3D_float pointVec = lightPoint.myPos.makeAVector(definingGlobalPoints[0].getPosRef());
        float mainDeterminant = mainNormal.dotProduct(lightPoint.headingVec);
        Vector3D_float secondaryNormal = pointVec.crossProduct3D(lightPoint.headingVec);
        if (mainDeterminant > -0.01) {
            return;
        }
        float countingNum = 1.0f / mainDeterminant;
        float tV = mainNormal.dotProduct(pointVec) * countingNum;
        if (tV > 0.01) {
            float determinantP1 = secondaryNormal.dotProduct(lightingVectors[1]) * countingNum;
            if (determinantP1 > 0.01) {
                float determinantP2 = secondaryNormal.dotProduct(lightingVectors[0]) * countingNum;
                if (determinantP2 > 0.01 && (determinantP1 + determinantP2) <= 1) {
                    if (tV < lightPoint.highestLight) {
                        lightPoint.highestLight = tV;
                        lightPoint.maxIndexCur = curentPos;
                    }
                }
            }
        }
    }
};


class LightSource {
private:
    LightTypes myType;
    std::array<LightRay_float, 20> lightSources{};
    Position3D_float myPos;
    SimpleColor myColor;
    int rayNumber, objectNum;
    float intezity, lenghtDecay;
    float sourceHeight, sourceWidth;

    void spehereVec(float intezity, float lenghtDecay) {
        float fibonnaciAngle = M_PI * (3 - sqrt(5));
        float quickNum = 1.0f / (rayNumber - 1);
        for (int i = 0; i < rayNumber; i += 1) {
            float yPos = 1 - (2 * i * quickNum);
            float theta = i * fibonnaciAngle;
            float radius = sqrt(1.0f - yPos * yPos);

            lightSources[i] = LightRay_float(myPos, intezity, myColor, Vector3D_float(simple3D_Pos_float(cos(theta) * radius, yPos, sin(theta) * radius)), lenghtDecay, (lenghtDecay * 4096) * intezity);
        }
    }

    void polygonVec(float intezity, float lenghtDecay, float sourceHeight, float sourceWidth, Vector3D_float directionRight, Vector3D_float directionUp) {
        int numberA = std::round(std::sqrt((sourceHeight * rayNumber) / sourceWidth));
        int numberB = std::round(std::sqrt((sourceWidth * rayNumber) / sourceHeight));
        if (!((numberA * numberB) == rayNumber)) {
            if (numberA > numberB) {
                numberA = rayNumber - numberB;
            }
            else {
                numberB = rayNumber - numberA;
            }
        }
        float diffX = sourceHeight / numberA;
        float diffY = sourceWidth / numberB;
        Vector3D_float normal = directionRight.crossProduct3D(directionUp);

        for (int i = 0; i < numberB; i += 1) {
            float quickDiff = i * diffY;
            simple3D_Pos_float heightPos = simple3D_Pos_float(quickDiff * directionUp.myPos.x, quickDiff * directionUp.myPos.y, quickDiff * directionUp.myPos.z);
            for (int j = 0; j < numberA; j += 1) {
                float quickDiff2 = j * diffX;
                simple3D_Pos_float rightPos = simple3D_Pos_float(quickDiff2 * directionRight.myPos.x, quickDiff2 * directionRight.myPos.y, quickDiff2 * directionRight.myPos.z);
                Position3D_float onePos = Position3D_float(myPos.changedBy(simple3D_Pos_float(heightPos.x + rightPos.x, heightPos.y + rightPos.y, heightPos.z + rightPos.z)));
                lightSources[(i * numberA) + j] = LightRay_float(onePos, intezity, myColor, normal, lenghtDecay, (lenghtDecay * 4096) * intezity);
            }
        }
    }

public:

    LightSource(LightTypes lightType, Position3D_float impPos, SimpleColor impCol,
        std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights, float intezity, float lenghtDecay,
        Vector3D_float directionRight = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float directionUp = Vector3D_float(simple3D_Pos_float(0,0,0)), float sourceHeight = 0, float sourceWidth = 0) {
        this->myType = lightType;
        this->myPos = impPos;
        this->rayNumber = 20;
        for (int i = 0; i < 20; i += 1) {
            this->lightSources[i] = (LightRay_float(impPos, 0, myColor, Vector3D_float(simple3D_Pos_float(0,0,0)), 0, 0));
        }
        this->myColor = impCol;
        this->intezity = intezity;
        this->lenghtDecay = lenghtDecay;
        this->sourceHeight = sourceHeight;
        this->sourceWidth = sourceWidth;

        this->objectNum = allPolygons.size();
        emitLightInit(allPolygons, allLights,directionRight, directionUp);
    }

    void resetLights(std::vector<GlobalPolygon_float> &allPolygons) {
        for (int i = 0; i < allPolygons.size(); i += 1) {
            allPolygons[i].rollBackColor();
        }
    }

    void lightingCycle(std::vector<GlobalPolygon_float> &allPolygons) {
        for (int i = 0; i < allPolygons.size(); i += 1) {
            if (allPolygons[i].reactToLight) {
                for (int j = 0; j < rayNumber; j += 1) {
                    allPolygons[i].lightingDuty(lightSources[j], i);
                }
            }
        }
        for (int i = 0; i < rayNumber; i += 1) {
            LightRay_float oneRay = lightSources[i];
            int oneIndex = oneRay.maxIndexCur;
            if (!(oneIndex == -1)) {
                allPolygons[oneIndex].changeLightColor(oneRay, oneRay.highestLight);
            }
        }
    }

    void emitLightInit(std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights, Vector3D_float directionRight = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float directionUp = Vector3D_float(simple3D_Pos_float(0,0,0))) {
        resetLights(allPolygons);
        if (myType == LightTypes::paralel) {
            polygonVec(intezity, lenghtDecay, sourceHeight, sourceWidth, directionRight, directionUp);
        }
        if (myType == LightTypes::pointLike) {
            spehereVec(intezity, lenghtDecay);
        }
        lightingCycle(allPolygons);
        for (LightSource &oneLight : allLights) {
            oneLight.lightingCycle(allPolygons);
        }
    }

    void emitLight(std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights, Vector3D_float directionRight = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float directionUp = Vector3D_float(simple3D_Pos_float(0,0,0))) {
        resetLights(allPolygons);
        if (myType == LightTypes::paralel) {
            polygonVec(intezity, lenghtDecay, sourceHeight, sourceWidth, directionRight, directionUp);
        }
        if (myType == LightTypes::pointLike) {
            spehereVec(intezity, lenghtDecay);
        }
        for (LightSource &oneLight : allLights) {
            oneLight.lightingCycle(allPolygons);
        }
    }

    void changePos(simple3D_Pos_float newPos, std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights, Vector3D_float directionRight = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float directionUp = Vector3D_float(simple3D_Pos_float(0,0,0))) {
        myPos.myPos = newPos;
        emitLight(allPolygons, allLights, directionRight, directionUp);
    }

    void movePos(simple3D_Pos_float newPos, std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights, Vector3D_float directionRight = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float directionUp = Vector3D_float(simple3D_Pos_float(0,0,0))) {
        myPos.myPos.x += newPos.x;
        myPos.myPos.y += newPos.y;
        myPos.myPos.z += newPos.z;
        emitLight(allPolygons, allLights, directionRight, directionUp);
    }

    void changeDirection(std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights, Vector3D_float directionRight = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float directionUp = Vector3D_float(simple3D_Pos_float(0,0,0))) {
        emitLight(allPolygons, allLights, directionRight, directionUp);
    }

    void changeSize(std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights, float newHeight, float newWidth, Vector3D_float directionRight = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float directionUp = Vector3D_float(simple3D_Pos_float(0,0,0))) {
        sourceHeight = newHeight;
        sourceWidth = newWidth;
        emitLight(allPolygons, allLights, directionRight, directionUp);
    }
};


struct playerHelpfulVals {
    float sinAngleY, cosAngleY, cosAngleZ, sinAngleZ;
    float angleY, angleZ;

    playerHelpfulVals(float angleY = 0, float angleZ = 0) {
        this->angleY = angleY;
        this->angleZ = angleZ;
        this->sinAngleY = std::sin(angleY);
        this->cosAngleY = std::cos(angleY);
        this->sinAngleZ = std::sin(angleZ);
        this->cosAngleZ = std::cos(angleZ);
    }

    void changeAngleY(float changeY) {
        angleY += changeY;
        sinAngleY = std::sin(angleY);
        cosAngleY = std::cos(angleY);
    }

    void changeAngleZ(float changeZ) {
        angleZ += changeZ;
        sinAngleZ = std::sin(angleZ);
        cosAngleZ = std::cos(angleZ);
    }
};


struct playerFullInfo {
    Vector3D_float headingVector = Vector3D_float(simple3D_Pos_float(0,0,0)), rightVector = Vector3D_float(simple3D_Pos_float(0,0,0)), upVector = Vector3D_float(simple3D_Pos_float(0,0,0));
    screenAndCameraInfo cameraInfo;
    Position3D_float originPoint;

    playerFullInfo(Vector3D_float headingVec = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float upVec = Vector3D_float(simple3D_Pos_float(0,0,0)),
        Vector3D_float rightVec = Vector3D_float(simple3D_Pos_float(0,0,0)), Position3D_float originPoint = Position3D_float(simple3D_Pos_float(0,0,0)),
        screenAndCameraInfo cameInfo = screenAndCameraInfo(0,0,0,0)) {
        this->headingVector = headingVec;
        this->rightVector = rightVec;
        this->upVector = upVec;
        this->cameraInfo = cameInfo;
        this->originPoint = originPoint;
    }

    void changeInfo(Vector3D_float &headingVec, Vector3D_float &upVec, Vector3D_float &rightVec, Position3D_float &neworiginPoint, screenAndCameraInfo &cameInfo) {
        headingVector = headingVec;
        rightVector = rightVec;
        upVector = upVec;
        cameraInfo = cameInfo;
        originPoint = neworiginPoint;
    }
};


struct LODInfo {
    bool blockify;
    float LODLevel;

    LODInfo(bool blocks, float level) {
        this->blockify = blocks;
        this->LODLevel = level;
    }
};


bool AABBCCColision(simple3D_Pos_float objB, simple3D_Pos_float &sizeA) {
    if (- sizeA.x < objB.x && sizeA.x > objB.x) {
        if (- sizeA.y < objB.y && sizeA.y > objB.y) {
            if (- sizeA.z < objB.z && sizeA.z > objB.z) {
                return true;
            }
        }
    }
    return false;
}


simple3D_Pos_float locilazePoint(Position3D_float &playerPos, Position3D_float &centrePoint, std::array<Vector3D_float, 3> &unitVectorsCentre) {
    Vector3D_float centPointVec = centrePoint.makeAVector(playerPos);
    return simple3D_Pos_float(unitVectorsCentre[0].dotProduct(centPointVec), unitVectorsCentre[1].dotProduct(centPointVec), unitVectorsCentre[2].dotProduct(centPointVec));
}


bool boundBoxCol(Position3D_float &playerPos, Position3D_float &centrePoint, std::array<Vector3D_float, 3> &unitVectorsCentre, simple3D_Pos_float &playerSize, simple3D_Pos_float &blockSize) {
    Position3D_float points[9] = {
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(playerSize.x, playerSize.y, playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(-playerSize.x, playerSize.y, playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(playerSize.x, -playerSize.y, playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(playerSize.x, playerSize.y, -playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(-playerSize.x, -playerSize.y, playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(playerSize.x, -playerSize.y, -playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(-playerSize.x, playerSize.y, -playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(-playerSize.x, -playerSize.y, -playerSize.z))),
        Position3D_float(playerPos.changedBy(simple3D_Pos_float(0, 0, 0)))
    };

    for (int i = 0; i < 9; i += 1) {
        if (AABBCCColision(locilazePoint(points[i], centrePoint, unitVectorsCentre), blockSize)) {
            return true;
        }
    }
    return false;
}


float findSmallestThree(std::array<float, 3> things) {
    float returningThing = things[0];
    for (int i = 0; i < 3; i += 1) {
        if (things[i] <= returningThing) {
            returningThing = things[i];
        }
    }
    return returningThing;
}


class Cube3D_float {
private:
    // links in style of having xxx and then another xxx ...
    // for now only cube collisions work

    std::array<int, 36> links{};
    std::array<Point_float, 8> points{};
    std::array<Position3D_float, 8> pseudoPos{};
    std::array<Position3D_float, 8> pseudoPosGradiant{};
    simple3D_Pos_float centrePoint;
    SimpleColor objectColor;
    int numsOfPoints;
    int numsOfFaces;
    bool stroked;
    SimpleColor outlineCol;
    simple3D_Pos_float objectSize;
    float outLineSize;
    float outerRad;
    float innerRad;
    std::array<Vector3D_float, 3> centreVec = {Vector3D_float(simple3D_Pos_float(1,0,0)), Vector3D_float(simple3D_Pos_float(0,1,0)), Vector3D_float(simple3D_Pos_float(0,0,1))};
    int firstPolygonNum;
    LODInfo myInfoLOD = LODInfo(true, 0.1);;

    void retypePolygonsThis(playerFullInfo &myInfo, std::vector<GlobalPolygon_float> &globalPolygons, std::vector<LightSource> &allLights) {
        for (int i = 0; i < 8; i += 1) {
            Position3D_float oneLocalPoint = points[i].getRelativePos(myInfo.headingVector, myInfo.upVector, myInfo.rightVector, myInfo.originPoint);
            float quickZ = 1.0f / pseudoPos[i].myPos.z;
            pseudoPos[i] = oneLocalPoint.makeIntoScreensCord(myInfo.cameraInfo, quickZ);
            pseudoPosGradiant[i] = oneLocalPoint.makeIntoGradiantable(myInfo.cameraInfo, quickZ);
        }

        int indexNum = firstPolygonNum;
        bool dontDrawOut = false;

        for (int i = 0; i < 36; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0.1 && pseudoPos[links[i+1]].myPos.z < 0.1 && pseudoPos[links[i+2]].myPos.z < 0.1) {
                dontDrawOut = true;
            }

            globalPolygons[indexNum].thisChange({points[links[i]], points[links[i+1]], points[links[i+2]]}, {pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]]},
                {pseudoPosGradiant[links[i]], pseudoPosGradiant[links[i+1]], pseudoPosGradiant[links[i+2]]}, dontDrawOut, myInfo.cameraInfo);

            globalPolygons[indexNum].rollBackColor();


            indexNum += 1;

            dontDrawOut = false;
        }
        if (allLights.size() > 0) {
            allLights[0].resetLights(globalPolygons);
            for (LightSource &oneLight : allLights) {
                oneLight.emitLight(globalPolygons, allLights);
            }
        }
    }

    void accualRotation(float cosAngleXY = 1, float cosAngleXZ = 1, float cosAngleYZ = 1, float sinAngleXY = 0, float sinAngleXZ = 0, float sinAngleYZ = 0) {
        centreVec[0] = Vector3D_float(simple3D_Pos_float(
                (centreVec[0].myPos.x * cosAngleXY - centreVec[0].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[0].myPos.z * sinAngleXZ,
                (centreVec[0].myPos.y * cosAngleXY + centreVec[0].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[0].myPos.z * sinAngleYZ,
                (centreVec[0].myPos.z * cosAngleXZ + centreVec[0].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[0].myPos.y * sinAngleYZ));

        centreVec[1] = Vector3D_float(simple3D_Pos_float(
                (centreVec[1].myPos.x * cosAngleXY - centreVec[1].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[1].myPos.z * sinAngleXZ,
                (centreVec[1].myPos.y * cosAngleXY + centreVec[1].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[1].myPos.z * sinAngleYZ,
                (centreVec[1].myPos.z * cosAngleXZ + centreVec[1].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[1].myPos.y * sinAngleYZ));

        centreVec[2] = Vector3D_float(simple3D_Pos_float(
                (centreVec[2].myPos.x * cosAngleXY - centreVec[2].myPos.y * sinAngleXY) * cosAngleXZ - centreVec[2].myPos.z * sinAngleXZ,
                (centreVec[2].myPos.y * cosAngleXY + centreVec[2].myPos.x * sinAngleXY) * cosAngleYZ - centreVec[2].myPos.z * sinAngleYZ,
                (centreVec[2].myPos.z * cosAngleXZ + centreVec[2].myPos.x * sinAngleXZ) * cosAngleYZ + centreVec[2].myPos.y * sinAngleYZ));

        for (int i = 0; i < numsOfPoints; i += 1) {
            // 1. XY rotation, 2. XZ rotation, 3. YZ rotation

            points[i].xVector = simple3D_Pos_float(
                (points[i].xVector.x * cosAngleXY - points[i].xVector.y * sinAngleXY) * cosAngleXZ - points[i].xVector.z * sinAngleXZ,
                (points[i].xVector.y * cosAngleXY + points[i].xVector.x * sinAngleXY) * cosAngleYZ - points[i].xVector.z * sinAngleYZ,
                (points[i].xVector.z * cosAngleXZ + points[i].xVector.x * sinAngleXZ) * cosAngleYZ + points[i].xVector.y * sinAngleYZ);

            points[i].yVector = simple3D_Pos_float(
                (points[i].yVector.x * cosAngleXY - points[i].yVector.y * sinAngleXY) * cosAngleXZ - points[i].yVector.z * sinAngleXZ,
                (points[i].yVector.y * cosAngleXY + points[i].yVector.x * sinAngleXY) * cosAngleYZ - points[i].yVector.z * sinAngleYZ,
                (points[i].yVector.z * cosAngleXZ + points[i].yVector.x * sinAngleXZ) * cosAngleYZ + points[i].yVector.y * sinAngleYZ);

            points[i].zVector = simple3D_Pos_float(
                (points[i].zVector.x * cosAngleXY - points[i].zVector.y * sinAngleXY) * cosAngleXZ - points[i].zVector.z * sinAngleXZ,
                (points[i].zVector.y * cosAngleXY + points[i].zVector.x * sinAngleXY) * cosAngleYZ - points[i].zVector.z * sinAngleYZ,
                (points[i].zVector.z * cosAngleXZ + points[i].zVector.x * sinAngleXZ) * cosAngleYZ + points[i].zVector.y * sinAngleYZ);
            updatePoses(i);

        }
    }

public:
    bool visibility;
    bool colision;
    bool reactToLight;

    Cube3D_float(int numOfPoints, int numFaces, std::array<Point_float, 8> points, simple3D_Pos_float &centre,
        std::array<int, 36> impLinks, std::vector<GlobalPolygon_float> &globalPolygons, int &globalPolygonsPos, std::vector<LightSource> &allLights, playerFullInfo &currentInfo,
        simple3D_Pos_float impSize = simple3D_Pos_float(0,0,0),
        SimpleColor objectColor = SimpleColor(0,0,0),bool blockify = true, float detail = 0.1,
        SimpleColor outlineCol = SimpleColor(0,0,0), bool stroked = false, float outLineSize = 1, bool visibility = false, bool colision = false, bool reactToLight = false) {
        this->outLineSize = outLineSize;
        this->stroked = stroked;
        this->outlineCol = outlineCol;
        this->objectColor = objectColor;
        this->numsOfFaces = numFaces;
        this->numsOfPoints = numOfPoints;
        this->centrePoint = centre;
        this->objectSize.x = impSize.x/2;
        this->objectSize.y = impSize.y/2;
        this->objectSize.z = impSize.z/2;
        this->links = impLinks;
        this->visibility = visibility;
        this->colision = colision;
        this->outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        this->innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
        this->centreVec[0] = Vector3D_float(simple3D_Pos_float(1,0,0));
        this->centreVec[1] = Vector3D_float(simple3D_Pos_float(0,1,0));
        this->centreVec[2] = Vector3D_float(simple3D_Pos_float(0,0,1));
        this->firstPolygonNum = globalPolygonsPos;
        this->reactToLight = reactToLight;
        this->myInfoLOD = LODInfo(blockify, detail);

        for (int i = 0; i < 8; i += 1) {
            this->points[i] = points[i];
            this->points[i].setupUnitVectors(centrePoint);
            this->pseudoPos[i] = Position3D_float(simple3D_Pos_float());
            this->pseudoPosGradiant[i] = Position3D_float(simple3D_Pos_float());
        }

        for (int i = 0; i < 12; i += 1) {
            globalPolygons.push_back(GlobalPolygon_float({Point_float(simple3D_Pos_float(0,0,0)),Point_float(simple3D_Pos_float(0,0,0)),Point_float(simple3D_Pos_float(0,0,0))},
                {Position3D_float(simple3D_Pos_float(0,0,0)),Position3D_float(simple3D_Pos_float(0,0,0)),Position3D_float(simple3D_Pos_float(0,0,0))}, {Position3D_float(simple3D_Pos_float(0,0,0)),Position3D_float(simple3D_Pos_float(0,0,0)),Position3D_float(simple3D_Pos_float(0,0,0))},
                objectColor, globalPolygonsPos, true, reactToLight));
            globalPolygonsPos += 1;
        }
        retypePolygonsThis(currentInfo, globalPolygons, allLights);

    }

    void retypePolygonsPlayer(playerFullInfo &myInfo, std::vector<GlobalPolygon_float> &globalPolygons) {
        for (int i = 0; i < 8; i += 1) {
            Position3D_float oneLocalPoint = points[i].getRelativePos(myInfo.headingVector, myInfo.upVector, myInfo.rightVector, myInfo.originPoint);
            float quickZ = 1.0f / pseudoPos[i].myPos.z;
            pseudoPos[i] = oneLocalPoint.makeIntoScreensCord(myInfo.cameraInfo, quickZ);
            pseudoPosGradiant[i] = oneLocalPoint.makeIntoGradiantable(myInfo.cameraInfo, quickZ);
        }

        int indexNum = firstPolygonNum;

        for (int i = 0; i < 36; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0.1 && pseudoPos[links[i+1]].myPos.z < 0.1 && pseudoPos[links[i+2]].myPos.z < 0.1) {
                indexNum += 1;
                continue;
            }

            globalPolygons[indexNum].playerChange({pseudoPos[links[i]], pseudoPos[links[i+1]], pseudoPos[links[i+2]]}, {pseudoPosGradiant[links[i]], pseudoPosGradiant[links[i+1]], pseudoPosGradiant[links[i+2]]},
                myInfo.cameraInfo, myInfoLOD.blockify , myInfoLOD.LODLevel, true);
            indexNum += 1;

        }
    }

    bool colide(Position3D_float &playerPos, simple3D_Pos_float &playerSize, float playerRad) {
        Position3D_float centrePosition3D = Position3D_float(centrePoint);
        float distance = std::pow(playerPos.myPos.x - centrePoint.x, 2) + std::pow(playerPos.myPos.y - centrePoint.y, 2) + std::pow(playerPos.myPos.z - centrePoint.z, 2);
        if (distance <= std::pow(playerRad + outerRad,2)) {
            if (distance <= std::pow(playerRad + innerRad,2)) {
                return true;
            }
            if (boundBoxCol(playerPos, centrePosition3D, centreVec, playerSize, objectSize)) {
                return true;
            }
        }
        return false;
    }

    void updatePoses(int i) {
        points[i].setPos(simple3D_Pos_float(
                centrePoint.x + (points[i].xVector.x * objectSize.x) + (points[i].yVector.x * objectSize.y) + (points[i].zVector.x * objectSize.z),
                centrePoint.y + (points[i].xVector.y * objectSize.x) + (points[i].yVector.y * objectSize.y) + (points[i].zVector.y * objectSize.z),
                centrePoint.z + (points[i].xVector.z * objectSize.x) + (points[i].yVector.z * objectSize.y) + (points[i].zVector.z * objectSize.z)));
    }

    void rotates(float angleYZ, float angleXZ, float angleXY, playerFullInfo &playInfo, std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights) {
        accualRotation(std::cos(angleXY), 1, 1, std::sin(angleXY));
        accualRotation(1, std::cos(angleXZ), 1, 0, std::sin(angleXZ));
        accualRotation(1, 1, std::cos(angleYZ), 0, 0, std::sin(angleYZ));
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    simple3D_Pos_float getSize() {
        return objectSize;
    }

    simple3D_Pos_float getPos() {
        return centrePoint;
    }

    void setPos(simple3D_Pos_float changePos, playerFullInfo &playInfo, std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights) {
        centrePoint = changePos;

        for (int i = 0; i < 8; i += 1) {
            updatePoses(i);
        }
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    void changePos(simple3D_Pos_float changePos, playerFullInfo &playInfo, std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights) {
        centrePoint.x += changePos.x;
        centrePoint.y += changePos.y;
        centrePoint.z += changePos.z;

        for (int i = 0; i < 8; i += 1) {
            updatePoses(i);
        }
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    void setSize(simple3D_Pos_float newSize, playerFullInfo &playInfo, std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights) {
        objectSize = newSize;
        for (int i = 0; i < 8; i += 1) {
            updatePoses(i);
        }
        outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    void changeSize(simple3D_Pos_float newSize, playerFullInfo &playInfo, std::vector<GlobalPolygon_float> &allPolygons, std::vector<LightSource> &allLights) {
        objectSize.x += newSize.x;
        objectSize.y += newSize.y;
        objectSize.z += newSize.z;
        for (int i = 0; i < 8; i += 1) {
            updatePoses(i);
        }
        outerRad = std::sqrt(std::pow(objectSize.x, 2) + std::pow(objectSize.y, 2) + std::pow(objectSize.z, 2));
        innerRad = findSmallestThree({objectSize.x, objectSize.y, objectSize.z});
        retypePolygonsThis(playInfo, allPolygons, allLights);
    }

    void drawOutFastSDL2(playerFullInfo &myInfo, uint8_t* colorsBuffer, uint8_t *zBuffer, std::vector<GlobalPolygon_float> &globalPolygons) {
        int indexNum = firstPolygonNum;

        for (int i = 0; i < 36; i += 3) {
            if (pseudoPos[links[i]].myPos.z < 0.1 && pseudoPos[links[i+1]].myPos.z < 0.1 && pseudoPos[links[i+2]].myPos.z < 0.1) {
                continue;
            }

            globalPolygons[indexNum].drawOutPolygonSDL2SuperFast(zBuffer, colorsBuffer, myInfo.cameraInfo, stroked, outLineSize);
            indexNum += 1;

        }
    }
};


void createCube(simple3D_Pos_float onePos, simple3D_Pos_float oneSize, SimpleColor objColor, SimpleColor outColor, bool stroked, std::vector<GlobalPolygon_float> &allPolygons, int &currentPolygon, std::vector<LightSource> &allLights,
    std::vector<Cube3D_float> &allObjects, playerFullInfo &currentPlayerInfo,
    float outlineSize = 10, bool colisions = false, bool visibility = false, bool blockify = true, float lodLevel = 0.1, bool lighted = false) {

    simple3D_Pos_float centre = simple3D_Pos_float(onePos.x + (oneSize.x/2), onePos.y + (oneSize.y/2), onePos.z + (oneSize.z/2));

    allObjects.push_back(Cube3D_float(8, 36, {
        Point_float(onePos.changedBy(0, 0, 0)),
        Point_float(onePos.changedBy(oneSize.x, 0, 0)),
        Point_float(onePos.changedBy(oneSize.x, oneSize.y, 0)),
        Point_float(onePos.changedBy(oneSize.x, oneSize.y, oneSize.z)),
        Point_float(onePos.changedBy(0, oneSize.y,oneSize.z)),
        Point_float(onePos.changedBy(0, 0, oneSize.z)),
        Point_float(onePos.changedBy(0, oneSize.y, 0)),
        Point_float(onePos.changedBy(oneSize.x, 0, oneSize.z))
    },
        centre, {
         0,2,1, 0,6,2,  // Z=0
         3,4,5, 3,5,7,  // Z=1
         0,5,4, 0,4,6,  // X=0
         1,2,3, 1,3,7,  // X=1
         0,1,7, 0,7,5,  // Y=0
         2,4,3, 2,6,4   // Y=1
    },allPolygons, currentPolygon, allLights, currentPlayerInfo, oneSize, objColor, blockify, lodLevel, outColor, stroked, outlineSize, visibility, colisions, lighted));
}


void createLight(std::vector<LightSource> &allLights, std::vector<GlobalPolygon_float> &allPolygons, LightTypes type, Position3D_float impPos, int rayNumber, SimpleColor color,
    float intenzity, float lenghtDecay, Vector3D_float directionRight = Vector3D_float(simple3D_Pos_float(0,0,0)), Vector3D_float directionUp = Vector3D_float(simple3D_Pos_float(0,0,0)),
    float sourceHeight = 0, float sourceWidth = 0) {
    allLights.push_back(LightSource(type, impPos, color, allPolygons, allLights, intenzity, lenghtDecay,directionRight, directionUp, sourceHeight, sourceWidth));
}



std::vector<GlobalPolygon_float> preparePolygonList() {
    std::vector<GlobalPolygon_float> newList;
    return newList;
}

std::vector<LightSource> prepareLightList() {
    std::vector<LightSource> newList;
    return newList;
}

std::vector<Cube3D_float> prepareObjectList() {
    std::vector<Cube3D_float> newList;
    return newList;
}


struct basicInfo {
    std::vector<GlobalPolygon_float> polygonList;
    int currePosPolygon = 0;
    std::vector<LightSource> lightSourcesList;
    std::vector<Cube3D_float> objectList;

    basicInfo() {
        this->polygonList = preparePolygonList();
        this->lightSourcesList = prepareLightList();
        this->objectList = prepareObjectList();
        this->currePosPolygon = 0;
    }
};


struct pressedKeys {
    bool forward, left, right, backward, up, down, cameraUp, cameraDown, cameraLeft, cameraRight;

    pressedKeys() {
        this->forward = false;
        this->left = false;
        this->right = false;
        this->backward = false;
        this->up = false;
        this->down = false;
        this->cameraUp = false;
        this->cameraDown = false;
        this->cameraLeft = false;
        this->cameraRight = false;
    }
};


class Player_float {
private:
    screenAndCameraInfo myCameraInfo;
    float speed;
    playerHelpfulVals values;
    playerHelpfulVals valuesUp;
    playerHelpfulVals valuesRight;
    Vector3D_float headingVec = Vector3D_float(simple3D_Pos_float());
    Vector3D_float rightVec = Vector3D_float(simple3D_Pos_float());
    Vector3D_float upVec = Vector3D_float(simple3D_Pos_float());
    bool pixelization;
    float detailLevel;
    float sizeRadius;
    bool colidingX = false;
    bool colidingY = false;
    bool colidingZ = false;
    simple3D_Pos_float lastPos;
    simple3D_Pos_float sizeBox;
    Position3D_float nextPositionX;
    Position3D_float nextPositionY;
    Position3D_float nextPositionZ;
    std::array<float, 3> changes;
    float gravity;
    bool jumpingMode;
    float sestivity;

    void linearColisionSetup() {
        colidingX = false;
        colidingY = false;
        colidingZ = false;
        nextPositionX = Position3D_float(simple3D_Pos_float(myPos.myPos.x + changes[0], myPos.myPos.y, myPos.myPos.z));
        nextPositionY = Position3D_float(simple3D_Pos_float(myPos.myPos.x, myPos.myPos.y + changes[1], myPos.myPos.z));
        nextPositionZ = Position3D_float(simple3D_Pos_float(myPos.myPos.x, myPos.myPos.y, myPos.myPos.z + changes[2]));
    }

public:
    Position3D_float myPos = Position3D_float(simple3D_Pos_float());
    playerFullInfo myBasicInfo;

    Player_float(float speed = 0.5, int screenHeight = 1080, int screenWidth = 1920, int fov = 90, simple3D_Pos_float beginPos = simple3D_Pos_float(0,0,0),
        bool pixelization = true, float detaiLevel = 1, simple3D_Pos_float sizeBox = simple3D_Pos_float(4,4,4), float gravity = 0, bool jumpingMode = false, float senstivity = 0.001) {
        this->myPos.setPosition(beginPos);
        this->speed = speed;
        this->pixelization = pixelization;
        this->detailLevel = detaiLevel;
        this->sizeRadius = findSmallestThree({sizeBox.x, sizeBox.y, sizeBox.z});
        this->sizeBox = sizeBox;
        this->colidingX = false;
        this->colidingY = false;
        this->colidingZ = false;
        this->nextPositionX = Position3D_float(beginPos);
        this->nextPositionY = Position3D_float(beginPos);
        this->nextPositionZ = Position3D_float(beginPos);
        this->changes = {0,0,0};
        this->gravity = gravity;
        this->jumpingMode = jumpingMode;
        this->sestivity = senstivity;

        this->values = playerHelpfulVals(0,0);
        this->valuesUp = playerHelpfulVals(this->values.angleY,(this->values.angleZ - 1.571f));
        this->valuesRight = playerHelpfulVals((this->values.angleY + 1.571f),this->values.angleZ);

        this->headingVec.setVector(simple3D_Pos_float(this->values.cosAngleY * this->values.cosAngleZ, this->values.sinAngleY * this->values.cosAngleZ, this->values.sinAngleZ));
        this->rightVec.setVector(simple3D_Pos_float(this->valuesRight.cosAngleY * this->valuesRight.cosAngleZ, this->valuesRight.sinAngleY * this->valuesRight.cosAngleZ, 0));
        this->upVec.setVector(simple3D_Pos_float(this->valuesUp.cosAngleY * this->valuesUp.cosAngleZ, this->valuesUp.sinAngleY * this->valuesUp.cosAngleZ, this->valuesUp.sinAngleZ));

        float radiansFOV = fov * (M_PI / 180);
        this->myCameraInfo = screenAndCameraInfo(((screenWidth * 0.5) / (std::tan(radiansFOV * 0.5))) * 1.33, ((screenHeight * 0.5) / (std::tan(radiansFOV * 0.5))), screenHeight, screenWidth);
        this->myBasicInfo = playerFullInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
    }

    void camera(uint8_t* sdlBuffer, uint8_t* zBuffer, basicInfo &globalInfo) {
        linearColisionSetup();
        for (Cube3D_float &oneObject : globalInfo.objectList) {
            if (oneObject.visibility) {
                oneObject.drawOutFastSDL2(myBasicInfo, sdlBuffer, zBuffer, globalInfo.polygonList);
            }
            if (oneObject.colision) {
                if (oneObject.colide(nextPositionX, sizeBox, sizeRadius)) {
                    colidingX = true;
                }
                if (oneObject.colide(nextPositionY, sizeBox, sizeRadius)) {
                    colidingY = true;
                }
                if (oneObject.colide(nextPositionZ, sizeBox, sizeRadius)) {
                    colidingZ = true;
                }
            }
        }
    }

    void cameraMovementSDL2(basicInfo &globalInfo, pressedKeys &keysPressed) {
         if (keysPressed.cameraDown) {
            if (values.angleZ >= -1.4) {
                values.changeAngleZ(-0.05);
                valuesUp.changeAngleZ(-0.05);

                headingVec.setVector(simple3D_Pos_float(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
                rightVec.setVector(simple3D_Pos_float(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
                upVec.setVector(simple3D_Pos_float(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
                myBasicInfo.changeInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
                for (Cube3D_float &oneObject : globalInfo.objectList) {
                    oneObject.retypePolygonsPlayer(myBasicInfo, globalInfo.polygonList);
                }
            }
        }
        if (keysPressed.cameraUp) {
            if (values.angleZ <= 1) {
                values.changeAngleZ(0.05);
                valuesUp.changeAngleZ(0.05);

                headingVec.setVector(simple3D_Pos_float(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
                rightVec.setVector(simple3D_Pos_float(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
                upVec.setVector(simple3D_Pos_float(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
                myBasicInfo.changeInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
                for (Cube3D_float &oneObject : globalInfo.objectList) {
                    oneObject.retypePolygonsPlayer(myBasicInfo, globalInfo.polygonList);
                }
            }
        }
        if (keysPressed.cameraLeft) {
            values.changeAngleY(-0.05);
            valuesUp.changeAngleY(-0.05);
            valuesRight.changeAngleY(-0.05);

            headingVec.setVector(simple3D_Pos_float(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
            rightVec.setVector(simple3D_Pos_float(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
            upVec.setVector(simple3D_Pos_float(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
            myBasicInfo.changeInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
            for (Cube3D_float &oneObject : globalInfo.objectList) {
                oneObject.retypePolygonsPlayer(myBasicInfo, globalInfo.polygonList);
            }
        }
        if (keysPressed.cameraRight) {
            values.changeAngleY(0.05);
            valuesUp.changeAngleY(0.05);
            valuesRight.changeAngleY(0.05);

            headingVec.setVector(simple3D_Pos_float(values.cosAngleY * values.cosAngleZ, values.sinAngleY * values.cosAngleZ, values.sinAngleZ));
            rightVec.setVector(simple3D_Pos_float(valuesRight.cosAngleY * valuesRight.cosAngleZ, valuesRight.sinAngleY * valuesRight.cosAngleZ, 0));
            upVec.setVector(simple3D_Pos_float(valuesUp.cosAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleY * valuesUp.cosAngleZ, valuesUp.sinAngleZ));
            myBasicInfo.changeInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
            for (Cube3D_float &oneObject : globalInfo.objectList) {
                oneObject.retypePolygonsPlayer(myBasicInfo, globalInfo.polygonList);
            }
        }
    }

    void defaultMovementSDL2(pressedKeys &keysPressed) {
        changes[0] = 0;
        changes[1] = 0;
        changes[2] = 0;
        if (keysPressed.forward) {
            changes[0] += speed * values.cosAngleY;
            changes[1] += speed * values.sinAngleY;
        }
        if (keysPressed.backward) {
            changes[0] += -speed * values.cosAngleY;
            changes[1] += -speed * values.sinAngleY;
        }
        if (keysPressed.right) {
            changes[0] += speed * valuesRight.cosAngleY;
            changes[1] += speed * valuesRight.sinAngleY;
        }
        if (keysPressed.left) {
            changes[0] += -speed * valuesRight.cosAngleY;
            changes[1] += -speed * valuesRight.sinAngleY;
        }
        if (keysPressed.up) {
            changes[2] += speed;
        }
        if (keysPressed.down) {
            changes[2] += -speed;
        }
    }

    void gravityMovement(pressedKeys &keysPressed) {
        changes[0] = 0;
        changes[1] = 0;
        if (std::abs(changes[2]) <= gravity * -10) {
            changes[2] += gravity;
        }
        else {
            changes[2] = gravity * 10;
        }
        if (keysPressed.forward) {
            changes[0] += speed * values.cosAngleY;
            changes[1] += speed * values.sinAngleY;
        }
        if (keysPressed.backward) {
            changes[0] += -speed * values.cosAngleY;
            changes[1] += -speed * values.sinAngleY;
        }
        if (keysPressed.right) {
            changes[0] += speed * valuesRight.cosAngleY;
            changes[1] += speed * valuesRight.sinAngleY;
        }
        if (keysPressed.left) {
            changes[0] += -speed * valuesRight.cosAngleY;
            changes[1] += -speed * valuesRight.sinAngleY;
        }
        if (keysPressed.up && colidingZ) {
            changes[2] += speed*20;
        }
    }

    void movementSDL2(basicInfo &globalInfo, pressedKeys &keysPressed) {
        if (!colidingX) {
            myPos.myPos.x += changes[0];
        }
        if (colidingX) {
            changes[0] = 0;
        }

        if (!colidingY) {
            myPos.myPos.y += changes[1];
        }
        if (colidingY) {
            changes[1] = 0;
        }

        if (!colidingZ) {
            myPos.myPos.z += changes[2];
        }
        if (colidingZ) {
            changes[2] = 0;
        }

        if (!colidingX || !colidingY || !colidingZ) {
            myBasicInfo.changeInfo(headingVec, upVec, rightVec, myPos, myCameraInfo);
            for (Cube3D_float &oneObject : globalInfo.objectList) {
                oneObject.retypePolygonsPlayer(myBasicInfo, globalInfo.polygonList);
            }
        }

        if (!jumpingMode) {
            defaultMovementSDL2(keysPressed);
        }
        if (jumpingMode) {
            gravityMovement(keysPressed);
        }
    }
};


void setIO(std::vector<gpio_num_t> outputIO, std::vector<gpio_num_t> inputy) {
    gpio_config_t output_conf = {};
    output_conf.intr_type = GPIO_INTR_DISABLE;
    output_conf.mode = GPIO_MODE_OUTPUT;
    for (auto outoo : outputIO) {
        output_conf.pin_bit_mask |= 1ULL << outoo;
    }
    output_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    output_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&output_conf);
    gpio_config_t inputs_conf = {};
    inputs_conf.intr_type = GPIO_INTR_DISABLE;
    inputs_conf.mode = GPIO_MODE_INPUT;
    for (auto intup : inputy) {
        output_conf.pin_bit_mask |= 1ULL << intup;
    }
    inputs_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    inputs_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&inputs_conf);
}


struct VGATimings {
    int screenWidth, screenHeight;
    int HFront, HBack, Hsync;
    int VFront, VBack, Vsync;
    int pxClock;
    bool hsyncPos, vsyncPos;
    uint8_t bitHsync, bitVsync;
    float VsyncFreq, HsyncFreq;
    int totalHeights, totalWidths;
    int precalcTotalHsync, precalcSum;

    int heightTotal() {
        return screenHeight + VBack + VFront + Vsync;
    }

    int widthTotal() {
        return screenWidth + HBack + HFront + Hsync;
    }

    //code: (vsync, hsync, r1, r2, g1, g2, b1, b2)
    void setupPins(std::array<int, 8> &pins) {
        for (int i = 0; i < 8; i += 1) {
            gpio_pad_select_gpio(pins[i]);
            gpio_set_direction((gpio_num_t)pins[i], GPIO_MODE_OUTPUT);
            gpio_matrix_out(pins[i], LCD_DATA_OUT0_IDX + i, false,false);
        }
    }

    void prepareLCDMod() {
        periph_module_enable(PERIPH_LCD_CAM_MODULE);
        periph_module_reset(PERIPH_LCD_CAM_MODULE);
        LCD_CAM.lcd_user.lcd_reset = 1;
        LCD_CAM.lcd_user.lcd_reset = 0;
        LCD_CAM.lcd_clock.val = 0;
        LCD_CAM.lcd_clock.clk_en = 1;
        LCD_CAM.lcd_clock.lcd_clk_sel = 2;      // Zdroj 160 MHz (PLL_D2_CLK)
        LCD_CAM.lcd_clock.lcd_clkm_div_num = 6;  // Celá část děličky
        LCD_CAM.lcd_clock.lcd_clkm_div_b = 2;    // Čitatel (0.4 = 2/5)
        LCD_CAM.lcd_clock.lcd_clkm_div_a = 5;    // Jmenovatel
        LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;
        LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
        LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
        LCD_CAM.lcd_clock.lcd_clkcnt_n = 0;
        LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
        LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;
        LCD_CAM.lcd_ctrl.val = 0;
        LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
        LCD_CAM.lcd_ctrl2.val = 0;
        LCD_CAM.lcd_ctrl2.lcd_hs_blank_en = 0;
        LCD_CAM.lcd_ctrl2.lcd_hsync_width = Hsync;
        LCD_CAM.lcd_ctrl2.lcd_vsync_width = Vsync;
        LCD_CAM.lcd_ctrl2.lcd_hsync_idle_pol = hsyncPos;
        LCD_CAM.lcd_ctrl2.lcd_vsync_idle_pol = vsyncPos;
        LCD_CAM.lcd_user.val = 0;
        LCD_CAM.lcd_user.lcd_2byte_en = 0;
        LCD_CAM.lcd_user.lcd_bit_order = 0;
        LCD_CAM.lcd_user.lcd_byte_order = 0;
        LCD_CAM.lcd_user.lcd_8bits_order = 1;
        LCD_CAM.lcd_user.lcd_always_out_en = 1;
        LCD_CAM.lcd_user.lcd_dout = 1;
        LCD_CAM.lcd_user.lcd_dout_cyclelen = 0;
    }

    VGATimings() {
        this->screenHeight = 480;
        this->screenWidth = 640;
        this->Hsync = 96;
        this->HFront = 16;
        this->HBack = 48;
        this->Vsync = 2;
        this->VBack = 33;
        this->VFront = 10;
        this->pxClock = 25000000;
        this->hsyncPos = true;
        this->vsyncPos = true;
        this->bitHsync = (1 << 6);
        this->bitVsync = (1 << 7);
        this->HsyncFreq = ((float)pxClock / widthTotal());
        this->VsyncFreq = (HsyncFreq / heightTotal());
        this->totalHeights = heightTotal();
        this->totalWidths = widthTotal();
        this->precalcTotalHsync = 800 - 96;
        this->precalcSum = 96 + 640 + 48;
    }
};


class gameInfo {
private:
    uint8_t* zBuffer;
    uint8_t* frontBuffer;
    uint8_t* backBuffer;
    uint8_t* linesBuffer[4];
    VGATimings myVGA;
    pressedKeys myKeys;
    gdma_channel_handle_t dma_chan;
    std::array<int, 8> pins;
    uint8_t fillerHsync = 0b10000111;
    uint8_t fillerZero = 0b11000111;
    uint8_t vSyncfillerHsync = 0b00000111;
    uint8_t vSyncfillerZero = 0b01000111;
    bool doubled;

    void prepareBlankLines() {
        for (int i = 0; i < 2; i += 1) {
            memset(linesBuffer[i], 0b11000000, myVGA.totalWidths);
            memset(linesBuffer[i], 0b10000000, myVGA.Hsync);
        }
        memset(linesBuffer[2], 0b01000000, myVGA.totalWidths);
        memset(linesBuffer[2], 0b00000000, myVGA.Hsync);

        memset(linesBuffer[3], 0b11000000, myVGA.totalWidths);
        memset(linesBuffer[3], 0b10000000, myVGA.Hsync);
    }

    void setupGDMA_Chan() {
        periph_module_reset(PERIPH_GDMA_MODULE);
        gdma_channel_alloc_config_t out_alloc_config = {
            .sibling_chan = NULL,
            .direction = GDMA_CHANNEL_DIRECTION_TX,
            .flags = {.reserve_sibling = 0}
        };

        gdma_new_channel(&out_alloc_config, &dma_chan);
        gdma_disconnect(dma_chan);
        gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
        gdma_start(dma_chan, (intptr_t)&dmaDesc[0]);
        gdma_transfer_ability_t ability = {
            .sram_trans_align = 4,
            .psram_trans_align = 0
        };
        gdma_set_transfer_ability(dma_chan, &ability);
        gdma_tx_event_callbacks_t myCallbacks = {
            .on_trans_eof = interuptGDMA_callback
        };
        gdma_register_tx_event_callbacks(dma_chan, &myCallbacks, this);
    }

    Player_float createBasicPlayer(int fov = 90, float senstivity = 0.001, float speed = 0.2, float gravity = 0, bool gravityMode = false, simple3D_Pos_float beginPos = simple3D_Pos_float(0,0,0),
    simple3D_Pos_float colisionBox = simple3D_Pos_float(4,4,4)) {
        return Player_float(speed, height, width, fov, beginPos, blockify, lodLevel, colisionBox, gravity, gravityMode, senstivity);
    }

public:

    uint8_t renderDistance;
    basicInfo gameGlobals;
    int height, width;
    uint8_t backgroundColor;
    bool blockify;
    float lodLevel;
    SemaphoreHandle_t mySemaphore;
    volatile int virLineCount = 0;
    int activeBuffer = 0;
    int activeStart;
    int activeEnd;
    pressedKeys currentKeys;
    lldesc_t* dmaDesc;
    Player_float myPlayer;

    gameInfo(int windowWidth = 320, int windowHeight = 240, std::array<int, 8> pins = {4, 5, 6, 7, 38, 39, 17, 18},
        uint8_t renderDistance = 255, SimpleColor backgroundColor = SimpleColor(0,0,255), bool blockify = true, float lodLevel = 0.5) {
        this->width = windowWidth;
        this->height = windowHeight;
        this->myVGA = VGATimings();
        this->virLineCount = 0;
        this->pins = pins;
        this->currentKeys = pressedKeys();
        this->mySemaphore = xSemaphoreCreateBinary();

        if (mySemaphore == NULL) {
            std::cout << "!!!!! Semaphore hasn`t been inicialized !!!!!!!" << std::endl;
            Serial.printf("!!!!! Semaphore hasn`t been inicialized !!!!!!!");
        }

        this->frontBuffer = (uint8_t *)heap_caps_malloc(windowWidth * windowHeight * sizeof(uint8_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        this->backBuffer = (uint8_t *)heap_caps_malloc(windowWidth * windowHeight * sizeof(uint8_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        this->linesBuffer[0] = (uint8_t *)heap_caps_malloc(myVGA.totalWidths, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        this->linesBuffer[1] = (uint8_t *)heap_caps_malloc(myVGA.totalWidths, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        this->linesBuffer[2] = (uint8_t *)heap_caps_malloc(myVGA.totalWidths, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        this->linesBuffer[3] = (uint8_t *)heap_caps_malloc(myVGA.totalWidths, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        this->zBuffer = (uint8_t*)heap_caps_malloc(windowHeight * windowWidth * sizeof(uint8_t), MALLOC_CAP_INTERNAL);
        this->backgroundColor = backgroundColor.convertToBinary();

        if (frontBuffer == NULL || zBuffer == NULL || linesBuffer[1] == NULL || linesBuffer[0] == NULL || linesBuffer[2] == NULL || linesBuffer[3] == NULL) {
            std::cout << "!!!!! Buffers haven't been inicialized !!!!!!!" << std::endl;
            Serial.printf("!!!!! Buffers haven't been inicialized !!!!!!!");
            return;
        }

        memset(frontBuffer, this->backgroundColor, height * width);
        memset(backBuffer, this->backgroundColor, height * width);
        memset(zBuffer, renderDistance, height * width);
        prepareBlankLines();

        dmaDesc = (lldesc_t*)heap_caps_malloc(2 * sizeof(lldesc_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

        for (int i = 0; i < 2; i += 1) {
            dmaDesc[i].size = myVGA.totalWidths;
            dmaDesc[i].length = myVGA.totalWidths;
            dmaDesc[i].owner = 1;
            dmaDesc[i].sosf = 0;
            dmaDesc[i].offset = 0;
            dmaDesc[i].eof = 1;
            dmaDesc[i].buf = linesBuffer[i];
            dmaDesc[i].qe.stqe_next = &dmaDesc[(i + 1) % 2];
        }

        Cache_WriteBack_Addr((uint32_t)dmaDesc, 2 * sizeof(lldesc_t));

        this->renderDistance = renderDistance;
        this->lodLevel = lodLevel;
        this->blockify = blockify;
        this->activeStart = myVGA.VBack + myVGA.Vsync;
        this->activeEnd = activeStart + myVGA.screenHeight;
        this->myPlayer = createBasicPlayer();
        this->doubled = false;
    }


    void IRAM_ATTR expandLines(uint8_t* bufferFill, int yPos) {
        int j = myVGA.Hsync + myVGA.HBack;
        uint8_t* pixelSource = &frontBuffer[yPos * width];
        uint32_t* fast32Bit = (uint32_t *)&bufferFill[j];
        //frontBuffer[currentRow + i] | 0b11000000
        //bufferFill[j++] = 0b11000100;
        //bufferFill[j++] = 0b11000100;
        for (int i = 0; i < width; i += 2) {
            //uint8_t pixel = frontBuffer[currentRow + i];
            uint8_t pixel1 = pixelSource[i];
            uint8_t pixel2 = pixelSource[i + 1];
            uint32_t doubelingNum = pixel2 << 24 | pixel2 << 16 | pixel1 << 8 | pixel1;
            fast32Bit[0] = doubelingNum | 0b11000000110000001100000011000000;
            fast32Bit += 1;
        }
    }

    void IRAM_ATTR interuptFunq() {
        virLineCount += 1;
        if (virLineCount >= myVGA.totalHeights) {
            virLineCount = 0;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(mySemaphore, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }

        if (virLineCount <= myVGA.Vsync) {
            dmaDesc[activeBuffer].buf = linesBuffer[2];
        }

        else if (virLineCount < activeEnd && virLineCount >= activeStart) {
            uint8_t* bufferFill = linesBuffer[activeBuffer];
            int yPos = (virLineCount - activeStart) >> 1;
            dmaDesc[activeBuffer].buf = linesBuffer[activeBuffer];
            expandLines(bufferFill, yPos);
            Cache_WriteBack_Addr((uint32_t)bufferFill, myVGA.totalWidths);
        }

        else {
            dmaDesc[activeBuffer].buf = linesBuffer[3];
        }
        Cache_WriteBack_Addr((uint32_t)dmaDesc, 2 * sizeof(lldesc_t));
        activeBuffer ^= 1;
    }

    static IRAM_ATTR bool interuptGDMA_callback(gdma_channel_handle_t dma_chanHandlerer, gdma_event_data_t *eventData, void *userData) {
        gameInfo* instance = (gameInfo*)userData;
        instance->interuptFunq();
        return true;
    }

    void vgaSetup() {
        setCpuFrequencyMhz(240);
        vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
        myVGA.prepareLCDMod();
        myVGA.setupPins(pins);
        setupGDMA_Chan();
        LCD_CAM.lc_dma_int_ena.lcd_trans_done_int_ena = 0;

        LCD_CAM.lcd_user.lcd_2byte_en = 0;
        LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;

        LCD_CAM.lcd_user.lcd_update = 1;
        LCD_CAM.lcd_user.lcd_start = 1;
    }

    void drawScene() {
        memset(backBuffer, backgroundColor, height * width);
        memset(zBuffer, renderDistance, height * width);

        myPlayer.camera(backBuffer, zBuffer, gameGlobals);
        playerMove();
    }

    void playerMove() {
        myPlayer.cameraMovementSDL2(gameGlobals, currentKeys);
    }

    void swapBuffers() {
        Cache_WriteBack_Addr((uint32_t)backBuffer, width * height);
        uint8_t* tempBuffer = backBuffer;
        backBuffer = frontBuffer;
        frontBuffer = tempBuffer;
    }

    ~gameInfo() {
        heap_caps_free(zBuffer);
        heap_caps_free(frontBuffer);
        heap_caps_free(backBuffer);
    }
};


static void core0Task(void* parameters) {
    vTaskDelay(50);
    pinMode(41, INPUT_PULLUP);
    pinMode(42, INPUT_PULLUP);
    gameInfo* instance = (gameInfo*)parameters;
    vTaskDelay(50);
    instance->vgaSetup();
    while (true) {
        if (digitalRead(42) == HIGH) {
            instance->currentKeys.cameraRight = true;
        }
        if (digitalRead(42) == LOW) {
            instance->currentKeys.cameraRight = false;
        }
        vTaskDelay(10);
    }
}


static void core1Task(void* parameters) {
    vTaskDelay(500);
    gameInfo* instance = (gameInfo*)parameters;
    vTaskDelay(500);
    while (true) {
        instance->drawScene();
        xSemaphoreTake(instance->mySemaphore, portMAX_DELAY);
        instance->swapBuffers();
        vTaskDelay(10);
    }
}


void createBasicCube(gameInfo &scene, Player_float &player, simple3D_Pos_float position = simple3D_Pos_float(0,0,0), simple3D_Pos_float size = simple3D_Pos_float(1,1,1), SimpleColor color = SimpleColor(0,0,0),
    SimpleColor outlineColor = SimpleColor(0,0,0), bool outline = false, float outlineSize = 10, bool colisions = false, bool visibility = true, bool reactToLight = false) {
    createCube(position, size, color, outlineColor, outline, scene.gameGlobals.polygonList, scene.gameGlobals.currePosPolygon, scene.gameGlobals.lightSourcesList, scene.gameGlobals.objectList,
        player.myBasicInfo, outlineSize, colisions, visibility, scene.blockify, scene.lodLevel, reactToLight);
}


void rotateBasicCube(gameInfo &scene, int cubeIndex, Player_float &player, float angleXY = 0, float angleXZ = 0, float angleYZ = 0) {
    scene.gameGlobals.objectList[cubeIndex].rotates(angleYZ, angleXZ, angleXY, player.myBasicInfo, scene.gameGlobals.polygonList, scene.gameGlobals.lightSourcesList);
}


void moveBasicCube(gameInfo &scene, int cubeIndex, Player_float &player, simple3D_Pos_float difference = simple3D_Pos_float(0,0,0)) {
    scene.gameGlobals.objectList[cubeIndex].changePos(difference,player.myBasicInfo, scene.gameGlobals.polygonList, scene.gameGlobals.lightSourcesList);
}


void setPosBasicCube(gameInfo &scene, int cubeIndex, Player_float &player, simple3D_Pos_float newPosition = simple3D_Pos_float(0,0,0)) {
    scene.gameGlobals.objectList[cubeIndex].setPos(newPosition,player.myBasicInfo, scene.gameGlobals.polygonList, scene.gameGlobals.lightSourcesList);
}


void changeSizeBasicCube(gameInfo &scene, int cubeIndex, Player_float &player, simple3D_Pos_float difference = simple3D_Pos_float(0,0,0)) {
    scene.gameGlobals.objectList[cubeIndex].changeSize(difference,player.myBasicInfo, scene.gameGlobals.polygonList, scene.gameGlobals.lightSourcesList);
}


void setSizeBasicCube(gameInfo &scene, int cubeIndex, Player_float &player, simple3D_Pos_float newSize = simple3D_Pos_float(1,1,1)) {
    scene.gameGlobals.objectList[cubeIndex].setSize(newSize,player.myBasicInfo, scene.gameGlobals.polygonList, scene.gameGlobals.lightSourcesList);
}


void createBasicLight(gameInfo &scene, simple3D_Pos_float position = simple3D_Pos_float(0,0,0), SimpleColor color = SimpleColor(255,255,255), int rayNumber = 15, float intenzity = 0.5, float lengDecay = 1) {
    createLight(scene.gameGlobals.lightSourcesList, scene.gameGlobals.polygonList, LightTypes::pointLike, position, rayNumber, color, intenzity, lengDecay);
}


void moveBasicLight(gameInfo &scene, int lightIndex, simple3D_Pos_float difference = simple3D_Pos_float(0,0,0)) {
    scene.gameGlobals.lightSourcesList[lightIndex].movePos(difference,scene.gameGlobals.polygonList, scene.gameGlobals.lightSourcesList);
}


void setPosBasicLight(gameInfo &scene, int lightIndex, simple3D_Pos_float newPos = simple3D_Pos_float(0,0,0)) {
    scene.gameGlobals.lightSourcesList[lightIndex].changePos(newPos,scene.gameGlobals.polygonList, scene.gameGlobals.lightSourcesList);
}


simple3D_Pos_float playerGetPos(Player_float &player) {
    return player.myPos.myPos;
}


gameInfo* game = nullptr;
Player_float* myPlayer = nullptr;


void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.printf("Starting ...");

    game = new gameInfo(320, 240);
    createBasicCube(*game, game->myPlayer, simple3D_Pos_float(-2,0,0));
    //createBasicCube(*game, game->myPlayer, simple3D_Pos_float(0,-2,0));
    //createBasicCube(*game, game->myPlayer, simple3D_Pos_float(0,0,-2));

    xTaskCreatePinnedToCore(
        core0Task,
        "vgaIntrupter", 4096*2, game, 24, nullptr, 0
    );

    xTaskCreatePinnedToCore(
        core1Task,
        "renderer", 4096*4, game, 24, nullptr, 1
    );
}


void loop() {
    vTaskDelay(1);
}