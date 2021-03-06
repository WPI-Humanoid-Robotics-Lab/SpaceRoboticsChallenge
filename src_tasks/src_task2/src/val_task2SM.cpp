#include <src_task2/val_task2.h>

// finite state machine name for tackling task-2
FSM(src_task2)
{
    //define the states
    FSM_STATES
    {
        STATE_PRE_INIT,
        STATE_INIT,
        STATE_DETECT_ROVER,
        STATE_WALK_TO_ROVER,
        STATE_DETECT_SOLAR_PANEL,
        STATE_GRASP_SOLAR_PANEL,
        STATE_PICK_SOLAR_PANEL,
        STATE_DETECT_SOLAR_ARRAY,
        STATE_WALK_TO_SOLAR_ARRAY,
        STATE_ROTATE_PANEL,
        STATE_FIND_CABLE_INTERMEDIATE,
        STATE_DETECT_SOLAR_ARRAY_FINE,
        STATE_ALIGN_TO_SOLAR_ARRAY,
        STATE_PLACE_SOLAR_PANEL_ON_GROUND,
        STATE_DETECT_DEPLOY_PANEL_BUTTON,
        STATE_DEPLOY_SOLAR_PANEL,
        STATE_DETECT_POWER_CABLE,
        STATE_PICKUP_POWER_CABLE,
        STATE_DETECT_SOCKET,
        STATE_PLUGIN_POWER_CABLE,
        STATE_DETECT_FINISH,
        STATE_WALK_TO_FINISH,
        STATE_END,
        STATE_ERROR,
        STATE_SKIP_CHECKPOINT,
        STATE_SKIP_TO_CP_1,
        STATE_SKIP_TO_CP_3,
        STATE_SKIP_TO_CP_4,
        STATE_SKIP_TO_CP_6,
        STATE_MANUAL_EXECUTION
    }

    // give the start state
    FSM_START(STATE_PRE_INIT);

    // state machine structure and logic, describe the states tansitions
    FSM_BGN // begin state machine
    {

        FSM_STATE(STATE_PRE_INIT)
        {
          // state excecution, call the task
          FSM_CALL_TASK(STATE_PRE_INIT)

           // state transitions
           FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/GOTO_STATE_INIT", FSM_NEXT(STATE_INIT))
            FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
          }
        }
        FSM_STATE(STATE_INIT)
        {
          // state excecution, call the task
          FSM_CALL_TASK(STATE_INIT)

           // state transitions
           FSM_TRANSITIONS
          {
            // transitions
            FSM_ON_EVENT("/INIT_RETRY", FSM_NEXT(STATE_INIT))
            FSM_ON_EVENT("/INIT_SUCESSFUL", FSM_NEXT(STATE_SKIP_CHECKPOINT))
            FSM_ON_EVENT("/INIT_FAILED", FSM_NEXT(STATE_ERROR))
            FSM_ON_EVENT("/SKIP_CHECK_POINT", FSM_NEXT(STATE_SKIP_CHECKPOINT))
            FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))

            // or on condition or next state directly
          }
        }
        FSM_STATE(STATE_DETECT_ROVER)
        {
            FSM_CALL_TASK(STATE_DETECT_ROVER)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DETECT_ROVER_RETRY", FSM_NEXT(STATE_DETECT_ROVER))
                FSM_ON_EVENT("/DETECTED_ROVER", FSM_NEXT(STATE_WALK_TO_ROVER))
                FSM_ON_EVENT("/DETECT_ROVER_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))

            }
        }
        FSM_STATE(STATE_WALK_TO_ROVER)
        {
            FSM_CALL_TASK(STATE_WALK_TO_ROVER)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/WALK_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/WALK_RETRY", FSM_NEXT(STATE_DETECT_ROVER))
                FSM_ON_EVENT("/WALK_EXECUTING", FSM_NEXT(STATE_WALK_TO_ROVER))
                FSM_ON_EVENT("/REACHED_ROVER", FSM_NEXT(STATE_DETECT_SOLAR_PANEL))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))

            }
        }
        FSM_STATE(STATE_DETECT_SOLAR_PANEL)
        {
            FSM_CALL_TASK(STATE_DETECT_SOLAR_PANEL)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DETECT_PANEL_RETRY", FSM_NEXT(STATE_DETECT_SOLAR_PANEL))
                FSM_ON_EVENT("/DETECTED_PANEL", FSM_NEXT(STATE_GRASP_SOLAR_PANEL))
                FSM_ON_EVENT("/DETECT_PANEL_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_GRASP_SOLAR_PANEL)
        {
            FSM_CALL_TASK(STATE_GRASP_SOLAR_PANEL)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/REDETECT_PANEL", FSM_NEXT(STATE_DETECT_SOLAR_PANEL))
                FSM_ON_EVENT("/GRASPED_PANEL", FSM_NEXT(STATE_PICK_SOLAR_PANEL))
                FSM_ON_EVENT("/GRASP_PANEL_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_PICK_SOLAR_PANEL)
        {
            FSM_CALL_TASK(STATE_PICK_SOLAR_PANEL)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/RE_REDETECT_PANEL", FSM_NEXT(STATE_DETECT_SOLAR_PANEL))
                FSM_ON_EVENT("/PICKED_PANEL", FSM_NEXT(STATE_DETECT_SOLAR_ARRAY))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_DETECT_SOLAR_ARRAY)
        {
            FSM_CALL_TASK(STATE_DETECT_SOLAR_ARRAY)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DETECT_ARRAY_RETRY", FSM_NEXT(STATE_DETECT_SOLAR_ARRAY))
                FSM_ON_EVENT("/DETECTED_ARRAY", FSM_NEXT(STATE_WALK_TO_SOLAR_ARRAY))  //change for automatic: STATE_WALK_TO_SOLAR_ARRAY
                FSM_ON_EVENT("/DETECT_ARRAY_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_WALK_TO_SOLAR_ARRAY)
        {
            FSM_CALL_TASK(STATE_WALK_TO_SOLAR_ARRAY)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/WALK_TO_ARRAY_RETRY", FSM_NEXT(STATE_DETECT_SOLAR_ARRAY))
                FSM_ON_EVENT("/WALK_TO_ARRAY_EXECUTING", FSM_NEXT(STATE_WALK_TO_SOLAR_ARRAY))
                FSM_ON_EVENT("/REACHED_ARRAY", FSM_NEXT(STATE_ROTATE_PANEL)) //change for automatic: STATE_ROTATE_PANEL
                FSM_ON_EVENT("/WALK_TO_ARRAY_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_ROTATE_PANEL)
        {
            FSM_CALL_TASK(STATE_ROTATE_PANEL)

            FSM_TRANSITIONS
            {
//                FSM_ON_EVENT("/ROTATE_PANEL_RETRY", FSM_NEXT(STATE_ROTATE_PANEL))
                FSM_ON_EVENT("/ROTATED_PANEL", FSM_NEXT(STATE_FIND_CABLE_INTERMEDIATE))
//                FSM_ON_EVENT("/ROTATE_PANEL_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_FIND_CABLE_INTERMEDIATE)
        {
            FSM_CALL_TASK(STATE_FIND_CABLE_INTERMEDIATE)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FIND_CABLE_RETRY", FSM_NEXT(STATE_FIND_CABLE_INTERMEDIATE))
                FSM_ON_EVENT("/FOUND_CABLE", FSM_NEXT(STATE_DETECT_SOLAR_ARRAY_FINE))
                FSM_ON_EVENT("/FIND_CABLE_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_DETECT_SOLAR_ARRAY_FINE)
        {
            FSM_CALL_TASK(STATE_DETECT_SOLAR_ARRAY_FINE)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DETECT_ARRAY_FINE_RETRY", FSM_NEXT(STATE_DETECT_SOLAR_ARRAY_FINE))
                FSM_ON_EVENT("/DETECTED_ARRAY_FINE", FSM_NEXT(STATE_ALIGN_TO_SOLAR_ARRAY)) //change for automatic: STATE_ALIGN_TO_SOLAR_ARRAY
                FSM_ON_EVENT("/DETECT_ARRAY_FINE_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_ALIGN_TO_SOLAR_ARRAY)
        {
            FSM_CALL_TASK(STATE_ALIGN_TO_SOLAR_ARRAY)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/ALIGN_TO_ARRAY_RETRY", FSM_NEXT(STATE_DETECT_SOLAR_ARRAY_FINE))
                FSM_ON_EVENT("/ALIGN_TO_ARRAY_EXECUTING", FSM_NEXT(STATE_ALIGN_TO_SOLAR_ARRAY))
                FSM_ON_EVENT("/ALIGNED_TO_ARRAY", FSM_NEXT(STATE_PLACE_SOLAR_PANEL_ON_GROUND))  //change for automatic: STATE_PLACE_SOLAR_PANEL_ON_GROUND
                FSM_ON_EVENT("/ALIGN_TO_ARRAY_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_PLACE_SOLAR_PANEL_ON_GROUND)
        {
            FSM_CALL_TASK(STATE_PLACE_SOLAR_PANEL_ON_GROUND)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/PLACE_ON_GROUND_RETRY", FSM_NEXT(STATE_PLACE_SOLAR_PANEL_ON_GROUND))
                FSM_ON_EVENT("/PLACED_ON_GROUND", FSM_NEXT(STATE_DETECT_DEPLOY_PANEL_BUTTON))  //change for automatic: STATE_DETECT_DEPLOY_PANEL_BUTTON
                FSM_ON_EVENT("/PLACED_ON_GROUND_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_DETECT_DEPLOY_PANEL_BUTTON)
        {
            FSM_CALL_TASK(STATE_DETECT_DEPLOY_PANEL_BUTTON)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DETECT_BUTTON_RETRY", FSM_NEXT(STATE_DETECT_DEPLOY_PANEL_BUTTON))
                FSM_ON_EVENT("/BUTTON_DETECTED", FSM_NEXT(STATE_DEPLOY_SOLAR_PANEL))
                FSM_ON_EVENT("/BUTTON_DETECTION_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_DEPLOY_SOLAR_PANEL)
        {
            FSM_CALL_TASK(STATE_DEPLOY_SOLAR_PANEL)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DETECT_AGAIN", FSM_NEXT(STATE_DETECT_DEPLOY_PANEL_BUTTON))
                FSM_ON_EVENT("/DEPLOY_RETRY", FSM_NEXT(STATE_DEPLOY_SOLAR_PANEL))
                FSM_ON_EVENT("/DEPLOYED", FSM_NEXT(STATE_DETECT_POWER_CABLE))
                FSM_ON_EVENT("/DEPLOY_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_DETECT_POWER_CABLE)
        {
            FSM_CALL_TASK(STATE_DETECT_POWER_CABLE)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DETECT_CABLE_RETRY", FSM_NEXT(STATE_DETECT_POWER_CABLE))
                FSM_ON_EVENT("/DETECTED_CABLE", FSM_NEXT(STATE_PICKUP_POWER_CABLE))  //change for automatic: STATE_PICKUP_POWER_CABLE
                FSM_ON_EVENT("/DETECT_CABLE_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_PICKUP_POWER_CABLE)
        {
            FSM_CALL_TASK(STATE_PICKUP_POWER_CABLE)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/PICKUPCABLE_RETRY", FSM_NEXT(STATE_DETECT_POWER_CABLE))
                FSM_ON_EVENT("/CABLE_PICKED", FSM_NEXT(STATE_DETECT_SOCKET))  //change for automatic: STATE_DETECT_SOCKET
                FSM_ON_EVENT("/PICKUP_CABLE_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_DETECT_SOCKET)
        {
            FSM_CALL_TASK(STATE_DETECT_SOCKET)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/DETECT_SOCKET_RETRY", FSM_NEXT(STATE_DETECT_SOCKET))
                FSM_ON_EVENT("/DETECTED_SOCKET", FSM_NEXT(STATE_MANUAL_EXECUTION))  //change for automatic: STATE_PLUGIN_POWER_CABLE
                FSM_ON_EVENT("/DETECT_SOCKET_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_PLUGIN_POWER_CABLE)
        {
            FSM_CALL_TASK(STATE_PLUGIN_POWER_CABLE)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/PLUGIN_CABLE_RETRY", FSM_NEXT(STATE_PLUGIN_POWER_CABLE))
                FSM_ON_EVENT("/CABLE_PLUGGED", FSM_NEXT(STATE_DETECT_FINISH))   //change for automatic: STATE_DETECT_FINISH
                FSM_ON_EVENT("/REDETECT_CABLE", FSM_NEXT(STATE_DETECT_POWER_CABLE))
                FSM_ON_EVENT("/PLUGIN_CABLE_FAILED", FSM_NEXT(STATE_ERROR))
                FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
            }
        }
        FSM_STATE(STATE_DETECT_FINISH)
        {

          FSM_CALL_TASK(STATE_DETECT_FINISH)

          FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/DETECT_FINISH_RETRY", FSM_NEXT(STATE_DETECT_FINISH))
            FSM_ON_EVENT("/DETECT_FINISH_FAILED", FSM_NEXT(STATE_ERROR))
            FSM_ON_EVENT("/DETECT_FINISH_SUCESSFUL", FSM_NEXT(STATE_WALK_TO_FINISH))  //change for automatic: STATE_WALK_TO_FINISH
            FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
          }
        }
        FSM_STATE(STATE_WALK_TO_FINISH)
        {

          FSM_CALL_TASK(STATE_WALK_TO_FINISH)

          FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/WALK_TO_FINISH_RETRY", FSM_NEXT(STATE_DETECT_FINISH))
            FSM_ON_EVENT("/WALK_TO_FINISH_ERROR", FSM_NEXT(STATE_ERROR))
            FSM_ON_EVENT("/WALK_TO_FINISH_SUCESSFUL", FSM_NEXT(STATE_END))  //change for automatic: STATE_END
            FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
          }
        }
        FSM_STATE(STATE_END)
        {

          FSM_CALL_TASK(STATE_END)

          FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/END_SUCESSFUL", FSM_NEXT(STATE_END))
          }
        }
        FSM_STATE(STATE_ERROR)
        {

          FSM_CALL_TASK(STATE_ERROR)

          FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/RESTART", FSM_NEXT(STATE_INIT))
            FSM_ON_EVENT("/SKIP_CHECK_POINT", FSM_NEXT(STATE_SKIP_CHECKPOINT))
            FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
          }
        }
        FSM_STATE(STATE_SKIP_CHECKPOINT)
        {

          FSM_CALL_TASK(STATE_SKIP_CHECKPOINT)

          FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/REGULAR", FSM_NEXT(STATE_DETECT_ROVER));
            FSM_ON_EVENT("/SKIP_TO_CP_3", FSM_NEXT(STATE_SKIP_TO_CP_3));
            FSM_ON_EVENT("/SKIP_TO_CP_4", FSM_NEXT(STATE_SKIP_TO_CP_4));
            FSM_ON_EVENT("/SKIP_TO_CP_6", FSM_NEXT(STATE_SKIP_TO_CP_6));
            FSM_ON_EVENT("/MANUAL_EXECUTION", FSM_NEXT(STATE_MANUAL_EXECUTION))
          }
        }
        FSM_STATE(STATE_SKIP_TO_CP_3)
        {

          FSM_CALL_TASK(STATE_SKIP_TO_CP_3)

          FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/SKIPPED_TO_CP_3", FSM_NEXT(STATE_FIND_CABLE_INTERMEDIATE));
            FSM_ON_EVENT("/SKIP_TO_CP_3_RETRY", FSM_NEXT(STATE_SKIP_CHECKPOINT));
            FSM_ON_EVENT("/SKIP_TO_CP_3_FAILED", FSM_NEXT(STATE_ERROR));
          }
        }
        FSM_STATE(STATE_SKIP_TO_CP_4)
        {

          FSM_CALL_TASK(STATE_SKIP_TO_CP_4)

          FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/SKIPPED_TO_CP_4", FSM_NEXT(STATE_FIND_CABLE_INTERMEDIATE));
            FSM_ON_EVENT("/SKIP_TO_CP_4_RETRY", FSM_NEXT(STATE_SKIP_CHECKPOINT));
            FSM_ON_EVENT("/SKIP_TO_CP_4_FAILED", FSM_NEXT(STATE_ERROR));
          }
        }
        FSM_STATE(STATE_SKIP_TO_CP_6)
        {

          FSM_CALL_TASK(STATE_SKIP_TO_CP_6)

          FSM_TRANSITIONS
          {
            FSM_ON_EVENT("/SKIPPED_TO_CP_6", FSM_NEXT(STATE_DETECT_FINISH));
            FSM_ON_EVENT("/SKIP_CP_6_RETRY", FSM_NEXT(STATE_SKIP_CHECKPOINT));
            FSM_ON_EVENT("/SKIP_CP_6_FAILED", FSM_NEXT(STATE_ERROR));
          }
        }

        FSM_STATE(STATE_MANUAL_EXECUTION)
        {

          FSM_CALL_TASK(STATE_MANUAL_EXECUTION)

          FSM_TRANSITIONS
          {
              FSM_ON_EVENT("/GOTO_STATE_INIT", FSM_NEXT(STATE_INIT));
              FSM_ON_EVENT("/GOTO_STATE_DETECT_ROVER", FSM_NEXT(STATE_DETECT_ROVER));
              FSM_ON_EVENT("/GOTO_STATE_WALK_TO_ROVER", FSM_NEXT(STATE_WALK_TO_ROVER));
              FSM_ON_EVENT("/GOTO_STATE_DETECT_SOLAR_PANEL", FSM_NEXT(STATE_DETECT_SOLAR_PANEL));
              FSM_ON_EVENT("/GOTO_STATE_GRASP_SOLAR_PANEL", FSM_NEXT(STATE_GRASP_SOLAR_PANEL));
              FSM_ON_EVENT("/GOTO_STATE_PICK_SOLAR_PANEL", FSM_NEXT(STATE_PICK_SOLAR_PANEL));
              FSM_ON_EVENT("/GOTO_STATE_DETECT_SOLAR_ARRAY", FSM_NEXT(STATE_DETECT_SOLAR_ARRAY));
              FSM_ON_EVENT("/GOTO_STATE_WALK_TO_SOLAR_ARRAY", FSM_NEXT(STATE_WALK_TO_SOLAR_ARRAY));
              FSM_ON_EVENT("/GOTO_STATE_ROTATE_PANEL", FSM_NEXT(STATE_ROTATE_PANEL));
              FSM_ON_EVENT("/GOTO_STATE_FIND_CABLE_INTERMEDIATE", FSM_NEXT(STATE_FIND_CABLE_INTERMEDIATE));
              FSM_ON_EVENT("/GOTO_STATE_DETECT_SOLAR_ARRAY_FINE", FSM_NEXT(STATE_DETECT_SOLAR_ARRAY_FINE));
              FSM_ON_EVENT("/GOTO_STATE_ALIGN_TO_SOLAR_ARRAY", FSM_NEXT(STATE_ALIGN_TO_SOLAR_ARRAY));
              FSM_ON_EVENT("/GOTO_STATE_PLACE_SOLAR_PANEL_ON_GROUND", FSM_NEXT(STATE_PLACE_SOLAR_PANEL_ON_GROUND));
              FSM_ON_EVENT("/GOTO_STATE_DETECT_DEPLOY_PANEL_BUTTON", FSM_NEXT(STATE_DETECT_DEPLOY_PANEL_BUTTON));
              FSM_ON_EVENT("/GOTO_STATE_DEPLOY_SOLAR_PANEL", FSM_NEXT(STATE_DEPLOY_SOLAR_PANEL));
              FSM_ON_EVENT("/GOTO_STATE_DETECT_POWER_CABLE", FSM_NEXT(STATE_DETECT_POWER_CABLE));
              FSM_ON_EVENT("/GOTO_STATE_PICKUP_POWER_CABLE", FSM_NEXT(STATE_PICKUP_POWER_CABLE));
              FSM_ON_EVENT("/GOTO_STATE_DETECT_SOCKET", FSM_NEXT(STATE_DETECT_SOCKET));
              FSM_ON_EVENT("/GOTO_STATE_PLUGIN_POWER_CABLE", FSM_NEXT(STATE_PLUGIN_POWER_CABLE));
              FSM_ON_EVENT("/GOTO_STATE_DETECT_FINISH", FSM_NEXT(STATE_DETECT_FINISH));
              FSM_ON_EVENT("/GOTO_STATE_WALK_TO_FINISH", FSM_NEXT(STATE_WALK_TO_FINISH));
              FSM_ON_EVENT("/GOTO_STATE_END", FSM_NEXT(STATE_END));
              FSM_ON_EVENT("/GOTO_STATE_ERROR", FSM_NEXT(STATE_ERROR));
              FSM_ON_EVENT("/GOTO_STATE_SKIP_CHECKPOINT", FSM_NEXT(STATE_SKIP_CHECKPOINT));
              FSM_ON_EVENT("/GOTO_STATE_SKIP_TO_CP_1", FSM_NEXT(STATE_SKIP_TO_CP_1));
              FSM_ON_EVENT("/GOTO_STATE_SKIP_TO_CP_3", FSM_NEXT(STATE_SKIP_TO_CP_3));
              FSM_ON_EVENT("/GOTO_STATE_SKIP_TO_CP_4", FSM_NEXT(STATE_SKIP_TO_CP_4));
              FSM_ON_EVENT("/GOTO_STATE_SKIP_TO_CP_6", FSM_NEXT(STATE_SKIP_TO_CP_6));
          }
        }
    }
    FSM_END
}
