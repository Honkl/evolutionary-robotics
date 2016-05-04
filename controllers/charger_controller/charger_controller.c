/*
 * File:          charger_controller.c
 * Date:          25.04.2016
 * Description:   A basic controller for Aseba-challenge charger.
 * Author:        Jan Kluj, Jakub Naplava, Ondrej Svec
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/display.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

int NUMBER_OF_CHARGERS = 2;
int CHARGERS_NAME_LENGTH = 8;
int EMITT_CHANNEL = 1;
int RECEIVE_CHANNEL = 2;
int energy = 5;
const char* name1 = "CHARGER1.BODY.SHAPE.APPEREANCE.MATERIAL";
const char* name2 = "CHARGER2.BODY.SHAPE.APPEREANCE.MATERIAL";

void set_color_field(const double* colors) {
  const char* chosenName = "";

  if(strcmp(wb_robot_get_name(), "CHARGER1") == 0) {
    chosenName = name1;
  } else if(strcmp(wb_robot_get_name(), "CHARGER2") == 0) {
    chosenName = name2;
  }

  WbNodeRef material = wb_supervisor_node_get_from_def(chosenName);

  WbFieldRef diffuse_color = wb_supervisor_node_get_field(material, "diffuseColor");
  wb_supervisor_field_set_sf_color(diffuse_color, colors);
}

void set_to_discharged() {
  const double INITIAL[3] = { 1, 0, 0 };
  set_color_field(INITIAL);
}

void set_to_charged() {
  const double INITIAL[3] = { 0, 0.7, 1 };
  set_color_field(INITIAL);
}

WbDeviceTag EMITTER;
WbDeviceTag RECEIVER;

void get_emitter() {
  EMITTER = wb_robot_get_device("emitter");
  wb_emitter_set_range(EMITTER, 0.15);

  int channel = wb_emitter_get_channel(EMITTER);
  if (channel != EMITT_CHANNEL) {
    wb_emitter_set_channel(EMITTER, EMITT_CHANNEL);
  }
}

void get_receiver() {
  RECEIVER = wb_robot_get_device("receiver");
  wb_receiver_enable(RECEIVER, TIME_STEP);
  int channel = wb_receiver_get_channel(RECEIVER);
  if (channel != RECEIVE_CHANNEL) {
    wb_receiver_set_channel(RECEIVER, RECEIVE_CHANNEL);
  }
}

int message_number = 0;

void emitt_message() {
  message_number++;
  char message[15];

  sprintf(message, "%d", message_number);
  //printf("Sending message ");
  //printf("%s\n", message);

  wb_emitter_send(EMITTER, message, strlen(message) + 1);
}

void receive_message() {
  /* is there at least one packet in the receiver's queue ? */
  if (wb_receiver_get_queue_length(RECEIVER) > 0) {
        /* read current packet's data */
      // const char *buffer = wb_receiver_get_data(RECEIVER);
      //printf("ACK Received\n"); //%s\"\n", buffer);
      energy--;
        /* fetch next packet */
      wb_receiver_next_packet(RECEIVER);
  }
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  get_emitter();
  get_receiver();

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */

  double time = wb_robot_get_time();
  double discharged_time = -1;

  while (wb_robot_step(TIME_STEP) != -1) {

    receive_message();

    double current_time = wb_robot_get_time();
    if (current_time - time > 1) {

      time = current_time;

      if (energy > 0) {
        emitt_message();
      }

      if (energy <= 0 && discharged_time == -1) {
        set_to_discharged();
        discharged_time = wb_robot_get_time();
      }

      if (discharged_time != -1) {
        // 5 seconds to recharge this charger
        if (wb_robot_get_time() - discharged_time > 5) {
            energy += 5;

            if (energy > 0) {
              set_to_charged();
            }

            discharged_time = -1;
        }
      }


      //printf("Energy: ");
      //printf("%d\n", energy);
    }
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
