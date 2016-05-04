/*
 * Description:  Default controller of the e-puck robot
 */

/* include headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/device.h>
#include <webots/camera.h>
#include <webots/led.h>
#include <webots/robot.h>
#include <webots/nodes.h>
#include "../advanced_genetic_algorithm_supervisor/genotype.h"
#include "../advanced_genetic_algorithm_supervisor/genotype.c"
#include "../advanced_genetic_algorithm_supervisor/random.h"
#include "../advanced_genetic_algorithm_supervisor/random.c"

/* Device stuff */
#define DISTANCE_SENSORS_NUMBER 8
#define CAMERA_WIDTH 4
#define CAMERA_HEIGHT 1
#define PROXIMITY_SENSORS 4
#define NUM_HIDDEN 6
#define NUM_OUTPUT 2
#define NUM_INPUT ((CAMERA_WIDTH * CAMERA_HEIGHT * 3) + PROXIMITY_SENSORS) // 16
#define GENOTYPE_SIZE ((NUM_INPUT*NUM_HIDDEN) + (NUM_HIDDEN*NUM_OUTPUT) + NUM_HIDDEN + NUM_OUTPUT)

static double W1[NUM_HIDDEN][NUM_INPUT];
static double W2[NUM_OUTPUT][NUM_HIDDEN];
static double b1[NUM_HIDDEN];
static double b2[NUM_OUTPUT];

static const char *GENOTYPE_FILE_NAME = "../advanced_genetic_algorithm_supervisor/genotype.txt";

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
#define TIME_STEP 256
static double speeds[2];

static int epuck_energy = 10;
static int EMITT_CHANNEL = 2;
static int RECEIVE_CHANNEL = 1;
static WbDeviceTag EMITTER;
static WbDeviceTag RECEIVER;

static void genotype_to_matrices(Genotype g){
    const double *genes = genotype_get_genes(g);
    int cur = 0;
    int i;
    int j;
    for(i=0;i<NUM_HIDDEN; i++)
    {
        for(j=0;j<NUM_INPUT; j++)
        {
            W1[i][j] = genes[cur++];
            //printf("W1[%d][%d]: %f", i, j, W1[i][j]);
        }
    }

    for(i=0;i<NUM_OUTPUT; i++)
    {
        for(j=0;j<NUM_HIDDEN; j++)
        {
            W2[i][j] = genes[cur++];
            //printf("W2[%d][%d]: %f", i, j, W2[i][j]);
        }
    }

    for(i=0;i<NUM_OUTPUT; i++)
    {
        b2[i] = genes[cur++];
        //printf("b2[%d]: %f", i, b2[i]);
    }

    for(i=0;i<NUM_HIDDEN; i++)
    {
        b1[i] = genes[cur++];
        //printf("b1[%d]: %f", i, b1[i]);
    }
}

static void write_fitness_to_file() {
  const char* file_name = "../advanced_genetic_algorithm_supervisor/fitness.txt";
  FILE *outfile = fopen(file_name, "w");
  fprintf(outfile, "%d", epuck_energy);
  fclose(outfile);
}

static void reload_genotype(){
  genotype_set_size(GENOTYPE_SIZE);
  FILE *infile = fopen(GENOTYPE_FILE_NAME, "r");
  if (! infile) {
    printf("unable to read %s\n", GENOTYPE_FILE_NAME);
    return;
  }
  Genotype genotype = genotype_create();
  genotype_fread(genotype, infile);
  fclose(infile);
  genotype_to_matrices(genotype);

  epuck_energy = 10;
  write_fitness_to_file();
}

static double * forwardPass(double input[NUM_INPUT]) {

    int row, column;
    double output_hidden[NUM_HIDDEN];
    for (column = 0; column < NUM_HIDDEN; column++) {
        output_hidden[column] = 0;
    }

    double * output = (double*) malloc(NUM_OUTPUT * sizeof(double));;
    for (column = 0; column < NUM_OUTPUT; column++) {
        output[column] = 0;
    }

    // x * W1
    for (row = 0; row < NUM_HIDDEN; row++) {
        for (column = 0; column < NUM_INPUT; column++) {
            output_hidden[row] = output_hidden[row] + W1[row][column]*input[column];
        }
    }

    // output_hidden = tanh( x * W1 + b1)
    for (column = 0; column < NUM_HIDDEN; column++) {
        output_hidden[column] = tanh(output_hidden[column] + b1[column]);
    }

    // output_hidden * W2
    for (row = 0; row < NUM_OUTPUT; row++) {
        for (column = 0; column < NUM_HIDDEN; column++) {
            output[row] = output[row] + W2[row][column]*output_hidden[column];
        }
    }

    // tanh( output_hidden * W2)
    for (column = 0; column < NUM_OUTPUT; column++) {
        output[column] = tanh(output[column] + b2[column]);
    }

    return output;
}

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
  double nn_input[NUM_INPUT];

  WbDeviceTag camera = wb_robot_get_device("camera");
  const unsigned char *image = wb_camera_get_image(camera);
  for (int x = 0; x < CAMERA_WIDTH; x++)
    for (int y = 0; y < CAMERA_HEIGHT; y++) {
      int r = wb_camera_image_get_red(image, CAMERA_WIDTH, x, y);
      int g = wb_camera_image_get_green(image, CAMERA_WIDTH, x, y);
      int b = wb_camera_image_get_blue(image, CAMERA_WIDTH, x, y);
      //printf("[%d, %d]: red=%d, green=%d, blue=%d",x, y, r, g, b);
      nn_input[3 * (y + x * CAMERA_HEIGHT)] = r / 255;
      nn_input[3 * (y + x * CAMERA_HEIGHT) + 1] = g / 255;
      nn_input[3 * (y + x * CAMERA_HEIGHT) + 2] = b / 255;
  }

  // + 4 front proximity sensors (ps6, ps7, ps0, ps1)
  // proximity sensors values are updated each tick in the main while cycle
  // proximity values are normalized to interval [0,1] (see get_sensor_input())
  nn_input[3 * CAMERA_HEIGHT * CAMERA_WIDTH] = distance_sensors_values[0]; //ps0
  nn_input[3 * CAMERA_HEIGHT * CAMERA_WIDTH + 1] = distance_sensors_values[1]; //ps1
  nn_input[3 * CAMERA_HEIGHT * CAMERA_WIDTH + 2] = distance_sensors_values[2]; //ps6
  nn_input[3 * CAMERA_HEIGHT * CAMERA_WIDTH + 3] = distance_sensors_values[3]; //ps7

  double* nn_result = forwardPass(nn_input);

  speeds[0] = nn_result[0] * MAX_SPEED;
  speeds[1] = nn_result[1] * MAX_SPEED;

  // printf("speed left %f speed right %f", speeds[0], speeds[1]);

  free(nn_result);
}

static void go_backwards() {
  wb_differential_wheels_set_speed(-MAX_SPEED, -MAX_SPEED);
  //passive_wait(0.2);
}

static void turn_left() {
  wb_differential_wheels_set_speed(-MAX_SPEED, MAX_SPEED);
  //passive_wait(0.2);
}

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

  int channel = wb_receiver_get_channel(RECEIVER);
  if (channel != RECEIVE_CHANNEL) {
    wb_receiver_set_channel(RECEIVER, RECEIVE_CHANNEL);
  }
}

static void emitt_message() {
  const char *message = "AKC";
  //printf("Sending ACK\n");
  wb_emitter_send(EMITTER, message, strlen(message) + 1);
}

static void receive_message() {
  /* is there at least one packet in the receiver's queue ? */
  while (wb_receiver_get_queue_length(RECEIVER) > 0) {

    /* read current packet's data */
  	const char *buffer = wb_receiver_get_data(RECEIVER);

  	if(strcmp(buffer, "RESET")==0){
  		reload_genotype();
  	} else {
      //printf("E-puck recharged %s\n", buffer);
      epuck_energy++;
      emitt_message();
      // printf("Epuck energy: %d\n", epuck_energy);
      write_fitness_to_file(); // TODO. After changing measure fitness move somewhere...
    }

    /* fetch next packet */
    wb_receiver_next_packet(RECEIVER);
  }
}

// static char* read_line_from_file(char* file_name) {
//   FILE *infile = fopen(file_name, "r");
//   if (! infile) {
//     printf("unable to read %s\n", file_name);
//     return "";
//   }

//   char line[256];
//   char* result = malloc(sizeof(char));
//   while (fgets(line, sizeof(line), infile)) {
//   }
//   fclose(infile);

//   result = &line;
//   return result;
// }



int main(int argc, char **argv) {
  wb_robot_init();

  init_devices();
  get_emitter();
  get_receiver();

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera,100);

  passive_wait(0.1);

  printf("E-puck robot controller started...\n");

  reload_genotype();

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
      //printf("%d\n", epuck_energy);
      time = current_time;
    }
  };

  return EXIT_SUCCESS;
}

