//
// Created by 25664 on 2026/5/29.
//

#include "detect.h"
static void Detect_Init(uint32_t time)
{
    //��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ� offlineTime onlinetime Priority
    uint16_t set_item[DETECT_LIST_LENGHT][3] =
    {

        {5, 10, 11},     // trigger 1
        {5, 10, 11},     // trigger 2
        {80, 3, 12},     // RC device
        {50, 10, 15},//cap_double
        {50, 10, 15},//cap0727
                    {5,10,11},
                    {5,10,11},
                    {5,10,11},
                    {5,10,11},
    };

    for (uint8_t i = 0; i < DETECT_LIST_LENGHT; i++)
    {
        Detect_List[i].Offline_Time = set_item[i][0];
        Detect_List[i].Online_Time = set_item[i][1];
        Detect_List[i].Priority = set_item[i][2];
        Detect_List[i].f_is_Data_Error = NULL;
        Detect_List[i].f_Solve_Lost = NULL;
        Detect_List[i].f_Solve_Data_Error = NULL;

        Detect_List[i].Enable = 1;
        Detect_List[i].Error_Exist = 1;
        Detect_List[i].is_Lost = 1;
        Detect_List[i].is_Data_Error = 1;
        Detect_List[i].frequency = 0.0f;
        Detect_List[i].New_Time = time;
        Detect_List[i].Last_Time = time;
        Detect_List[i].Lost_Time = time;
        Detect_List[i].Work_Time = time;
    }
    Detect_List[RC_TOE].f_is_Data_Error = RC_Data_is_Error;
    // Detect_List[RC_TOE].f_Solve_Lost = Solve_RC_Lost;
    Detect_List[RC_TOE].f_Solve_Data_Error = Solve_RC_Data_Error;
}
