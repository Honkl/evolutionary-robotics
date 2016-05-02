/*
 * Description:  Default controller of the e-puck robot
 */

/* include headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/device.h>
#include <webots/camera.h>
#include <webots/led.h>
#include <webots/robot.h>
#include <webots/nodes.h>

/* Device stuff */
#define DISTANCE_SENSORS_NUMBER 8
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
};

#define GROUND_SENSORS_NUMBER 3
static WbDeviceTag ground_sensors[GROUND_SENSORS_NUMBER];
static double ground_sensors_values[GROUND_SENSORS_NUMBER] = { 0.0, 0.0, 0.0 };
static const char *ground_sensors_names[GROUND_SENSORS_NUMBER] = {
  "gs0", "gs1", "gs2"
};

#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {
  "led0", "led1", "led2", "led3",
  "led4", "led5", "led6", "led7",
  "led8", "led9"
};

#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 1000.0
#define TIME_STEP 64
static double speeds[2];

/* Breitenberg stuff */
static double weights[DISTANCE_SENSORS_NUMBER][2] = {
  {-2.0, -1.6},
  {-2.0, -1.6},
  {-0.8, 0.8},
  {0.0, 0.0},
  {0.0, 0.0},
  {0.8, -0.8},
  {-1.2, -1.6},
  {-1.2, -1.6}
};
static double offsets[2] = {
  0.5*MAX_SPEED, 0.5*MAX_SPEED
};

static int epuck_energy = 10;
static int EMITT_CHANNEL = 2;
static int RECEIVE_CHANNEL = 1;

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int) wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while(start_time + sec > wb_robot_get_time());
}

static void init_devices() {
  int i;
  for (i=0; i<DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors[i]=wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }
  for (i=0; i<LEDS_NUMBER; i++)
    leds[i]=wb_robot_get_device(leds_names[i]);

  // silently initialize the ground sensors if they exists
  for (i=0; i<GROUND_SENSORS_NUMBER; i++)
    ground_sensors[i] = (WbDeviceTag) 0;
  int ndevices = wb_robot_get_number_of_devices();
  for (i=0; i<ndevices; i++) {
    WbDeviceTag dtag = wb_robot_get_device_by_index(i);
    const char *dname = wb_device_get_name(dtag);
    WbNodeType dtype = wb_device_get_node_type(dtag);
    if (dtype == WB_NODE_DISTANCE_SENSOR && strlen(dname) == 3 && dname[0] == 'g' && dname[1] == 's') {
      int id = dname[2] - '0';
      if (id >=0 && id < GROUND_SENSORS_NUMBER) {
        ground_sensors[id] = wb_robot_get_device(ground_sensors_names[id]);
        wb_distance_sensor_enable(ground_sensors[id], get_time_step());
      }
    }
  }

  step();
}

static void reset_actuator_values() {
  int i;
  for (i=0; i<2; i++)
    speeds[i] = 0.0;
  for (i=0; i<LEDS_NUMBER; i++)
    leds_values[i] = false;
}

static void get_sensor_input() {
  int i;
  for (i=0; i<DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);

    // scale the data in order to have a value between 0.0 and 1.0
    // 1.0 representing something to avoid, 0.0 representing nothing to avoid
    distance_sensors_values[i] /= 4096;
  }

  for (i=0; i<GROUND_SENSORS_NUMBER; i++) {
    if (ground_sensors[i])
      ground_sensors_values[i] = wb_distance_sensor_get_value(ground_sensors[i]);
  }
}

static bool cliff_detected() {
  return false;
  int i;
  for (i=0; i<GROUND_SENSORS_NUMBER; i++) {
    if (!ground_sensors[i])
      return false;
    if (ground_sensors_values[i] < 500.0) // default 500.0
      return true;
  }
  return false;
}

static void set_actuators() {
  int i;
  for (i=0; i<LEDS_NUMBER; i++)
    wb_led_set(leds[i],leds_values[i]);
  wb_differential_wheels_set_speed(speeds[LEFT], speeds[RIGHT]);
}

static void blink_leds() {
  static int counter = 0;
  counter++;
  leds_values[(counter/10)%LEDS_NUMBER] = true;
}

static void run_braitenberg() {
  int i, j;
  for (i=0; i<2; i++) {
    speeds[i] = 0.0;
    for (j=0; j<DISTANCE_SENSORS_NUMBER; j++)
      speeds[i] += distance_sensors_values[j] * weights[j][i];

    speeds[i] = offsets[i] + speeds[i]*MAX_SPEED;
    if (speeds[i] > MAX_SPEED)
      speeds[i] = MAX_SPEED;
    else if (speeds[i] < -MAX_SPEED)
      speeds[i] = -MAX_SPEED;
  }
}

static void go_backwards() {
  wb_differential_wheels_set_speed(-MAX_SPEED, -MAX_SPEED);
  //passive_wait(0.2);
}

static void turn_left() {
  wb_differential_wheels_set_speed(-MAX_SPEED, MAX_SPEED);
  //passive_wait(0.2);
}


static WbDeviceTag EMITTER;
static WbDeviceTag RECEIVER;

static void get_emitter() {
  EMITTER = wb_robot_get_device("emitter");
  wb_emitter_set_range(EMITTER, 0.3);



  int channel = wb_emitter_get_channel(EMITTER);
  if (channel != EMITT_CHANNEL) {
    wb_emitter_set_channel(EMITTER, EMITT_CHANNEL);
  }

}

static void get_receiver() {
  RECEIVER = wb_robot_get_device("receiver");
  wb_receiver_enable(RECEIVER, TIME_STEP);

  //WbFieldRef aperture = wb_supervisor_node_get_field(RECEIVER, "aperture");
  //wb_supervisor_field_set_sf_float(aperture, -1);

  int channel = wb_receiver_get_channel(RECEIVER);
  if (channel != RECEIVE_CHANNEL) {
    wb_receiver_set_channel(RECEIVER, RECEIVE_CHANNEL);
  }
}

static void emitt_message() {
  const char *message = "AKC";
  printf("Sending ACK\n");
  wb_emitter_send(EMITTER, message, strlen(message) + 1);
}

static void receive_message() {

  /* is there at least one packet in the receiver's queue ? */
  while (wb_receiver_get_queue_length(RECEIVER) > 0) {

        /* read current packet's data */
      const char *buffer = wb_receiver_get_data(RECEIVER);
      printf("E-puck recharged %s\n", buffer);
      epuck_energy++;
      emitt_message();

        /* fetch next packet */
      wb_receiver_next_packet(RECEIVER);
  }
}



int main(int argc, char **argv) {
  wb_robot_init();

  printf("E-puck robot controller started...\n");

  init_devices();

  get_emitter();
  get_receiver();

  wb_robot_set_data("hello");  
  char* x = wb_robot_get_data();
  printf("%s", x);

  
WbDeviceTag camera = wb_robot_get_device("camera");
wb_camera_enable(camera,100);

  double time = wb_robot_get_time();
  while (wb_robot_step(TIME_STEP) != -1) {

    reset_actuator_values();
    get_sensor_input();

    receive_message();

    blink_leds();
    if (cliff_detected()) {
      go_backwards();
      turn_left();
    } else {
      run_braitenberg();
    }
    set_actuators();

    double current_time = wb_robot_get_time();
    if (current_time - time > 1) {
      printf("%d\n", epuck_energy);
      time = current_time;
    }

    //step();
  };

  return EXIT_SUCCESS;
}

