#include "STD_Methods.h"
#include "stdlib.h"
#include <math.h>



bool isInRange(float newVal, float Reference, float deltaRange)
{
    //if newVal is with in + or - deltaRange of the Reference variable
    if(abs((int)Reference - (int)newVal) > deltaRange)
    {
        //not in range
        return false;
    }
    //yah! we are in range
    return true;
}
//Returns:
////      0 -> 
//bool Rich_isInRange(float newVal, float Reference, float deltaRange)
//{
//    //if newVal is with in + or - deltaRange of the Reference variable
//    if(((int)Reference - (int)newVal) > deltaRange)
//    {
//        //not in range
//        return false;
//    }
//    //yah! we are in range
//    return true;
//}