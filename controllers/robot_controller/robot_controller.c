/*
 * Description:   A controller which sends datas from the emitter to the
 *                receiver while the robots avoid the obstacles.
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <string.h>
#include <stdio.h>

#define SPEED 60
#define TIME_STEP 64
#define COMMUNICATION_CHANNEL 1

int main() {
  WbDeviceTag ds0, ds1, emitter, receiver;
  int channel;
  double left_speed, right_speed;
  double ds0_value, ds1_value;
  int RECEIVE_CHANNEL = 1;
  int EMITT_CHANNEL = 2;

  wb_robot_init();

  emitter = wb_robot_get_device("emitter");    
  channel = wb_emitter_get_channel(emitter);
  if (channel != EMITT_CHANNEL) {
    wb_emitter_set_channel(emitter, EMITT_CHANNEL);
  }
  wb_emitter_set_range(emitter, 0.2);

  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);
  channel = wb_receiver_get_channel(receiver);
  if (channel != RECEIVE_CHANNEL) {
    wb_emitter_set_channel(receiver, RECEIVE_CHANNEL);
  }
  /* get a handler to the distance sensors and enable them */
  ds0 = wb_robot_get_device("ds0");
  ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  while(wb_robot_step(TIME_STEP)!=-1) {

      /* is there at least one packet in the receiver's queue ? */
    if (wb_receiver_get_queue_length(receiver) > 0) {

      printf("I'VE RECEIVED SOMETHING!\n");

      /* read current packet's data */
      const char *buffer = wb_receiver_get_data(receiver);
      printf("Communicating: received \"%s\"\n",buffer);
       
      /* fetch next packet */
      wb_receiver_next_packet(receiver);   
      
      /* send null-terminated message */
      const char *message = "ACK";
      wb_emitter_send(emitter, message, strlen(message) + 1);
    }
    
    ds0_value = wb_distance_sensor_get_value(ds0);
    ds1_value = wb_distance_sensor_get_value(ds1);

    if (ds1_value > 500) {
      /*
       * If both distance sensors are detecting something, this means that
       * we are facing a wall. In this case we need to move backwards.
       */
      if (ds0_value > 200) {
        left_speed = -SPEED;
        right_speed = -SPEED / 2;
      //  left_speed = -SPEED / 2;
      //    right_speed = -SPEED;        
      } else {

        /*
         * we turn proportionnaly to the sensors value because the
         * closer we are from the wall, the more we need to turn.
         */
        left_speed = -ds1_value / 10;
        right_speed = (ds0_value / 10) + 5;
      }
    } else if (ds0_value > 500) {
      left_speed = (ds1_value / 10) + 5;
      right_speed = -ds0_value / 10;
    } else {

      /*
       * if nothing has been detected we can move forward at maximal speed.
       */
      left_speed = SPEED;
      right_speed = SPEED;
    }

    /* set the motor speeds. */
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }
  
  wb_robot_cleanup();

  return 0;
}
