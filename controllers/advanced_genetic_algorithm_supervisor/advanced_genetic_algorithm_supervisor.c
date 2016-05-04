//   Description:   Supervisor code for genetic algorithm

#include "genotype.h"
#include "population.h"
#include "random.h"
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/display.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* Device stuff */
#define DISTANCE_SENSORS_NUMBER 8
#define CAMERA_WIDTH 4
#define CAMERA_HEIGHT 1
#define PROXIMITY_SENSORS 4
#define NUM_HIDDEN 6
#define NUM_OUTPUT 2
#define NUM_INPUT ((CAMERA_WIDTH * CAMERA_HEIGHT * 3) + PROXIMITY_SENSORS) // 16
#define GENOTYPE_SIZE ((NUM_INPUT*NUM_HIDDEN) + (NUM_HIDDEN*NUM_OUTPUT) + NUM_HIDDEN + NUM_OUTPUT)
#define EVOLVE_TIME 120

static const int POPULATION_SIZE = 10;
static const int NUM_GENERATIONS = 25;
static const char *FILE_NAME = "fittest.txt";
static const char *GENOTYPE_FILE_NAME = "../advanced_genetic_algorithm_supervisor/genotype.txt";

// must match the values in the advanced_genetic_algorithm.c code
static const int NUM_SENSORS = 8;
static const int NUM_WHEELS  = 2;

// index access
enum { X, Y, Z };

static int time_step;
static WbDeviceTag emitter;   // to send genes to robot
static WbDeviceTag display;   // to display the fitness evolution
static int display_width, display_height;

// the GA population
static Population population;

// for reading or setting the robot's position and orientation
static WbFieldRef robot_translation;
static WbFieldRef robot_rotation;
static double robot_trans0[3];  // a translation needs 3 doubles
static double robot_rot0[4];    // a rotation needs 4 doubles

// start with a demo until the user presses the 'O' key
// (change this if you want)
static bool demo = false;

void draw_scaled_line(int generation, double y1, double y2) {
  const double XSCALE = (double)display_width / NUM_GENERATIONS;
  const double YSCALE = 10.0;
  wb_display_draw_line(display, (generation - 0.5) * XSCALE, display_height - y1 * YSCALE,
    (generation + 0.5) * XSCALE, display_height - y2 * YSCALE);
}

// plot best and average fitness
void plot_fitness(int generation, double best_fitness, double average_fitness) {
  static double prev_best_fitness = 0.0;
  static double prev_average_fitness = 0.0;
  if (generation > 0) {
    wb_display_set_color(display, 0xff0000); // red
    draw_scaled_line(generation, prev_best_fitness, best_fitness);

    wb_display_set_color(display, 0x00ff00); // green
    draw_scaled_line(generation, prev_average_fitness, average_fitness);
  }

  prev_best_fitness = best_fitness;
  prev_average_fitness = average_fitness;
}

// run the robot simulation for the specified number of seconds
void run_seconds(double seconds) {
  int i, n = 1000.0 * seconds / time_step;
  for (i = 0; i < n; i++) {
    wb_robot_step(time_step);
  }
}

void write_nn_to_file(Genotype genotype) {
  FILE *outfile = fopen(GENOTYPE_FILE_NAME, "w");

  // // genotype_get_genes(genotype), GENOTYPE_SIZE * sizeof(double)
  // char *genstr = (char*) genotype_get_genes(genotype);
  // for (int i = 0; i < GENOTYPE_SIZE*sizeof(double); ++i)
  // {
  // 	genstr[i] = genstr[i]+1;
  // }
  // fprintf(outfile, genstr);
  // fclose(outfile);

  if (outfile) {
    genotype_fwrite(genotype, outfile);
    fclose(outfile);
    printf("wrote best genotype into %s\n", GENOTYPE_FILE_NAME);
  }
  else {
    printf("unable to write %s\n", GENOTYPE_FILE_NAME);
  }
}

double read_fitness() {
  const char* file_name = "fitness.txt";
  FILE *infile = fopen(file_name, "r");
  if (! infile) {
    printf("unable to read %s\n", file_name);
    return -1;
  }
  double fitness = 0;
  fscanf(infile, "%lf", &fitness);
  fclose(infile);
  return fitness;
}

double measure_fitness() {
  return read_fitness();

  // const double *load_trans = wb_supervisor_field_get_sf_vec3f(load_translation);
  // double dx = load_trans[X] - load_trans0[X];
  // double dz = load_trans[Z] - load_trans0[Z];
  // return sqrt(dx * dx + dz * dz);
}

bool is_in_circle(double circle_x, double circle_y, double radius, double point_x, double point_y) {
  return (pow(point_x - circle_x, 2) + pow(point_y - circle_y, 2) <= pow(radius, 2));
}

// evaluate one genotype at a time
void evaluate_genotype(Genotype genotype) {

  // prepare neural network for e-puck
  write_nn_to_file(genotype);

  // reset robot position
  double x1 = 0.2;
  double z1 = 0.2;
  double x2 = -0.2;
  double z2 = -0.2;
  double radius = 0.05;

  double x = 0.8 * random_get_uniform() - 0.4;
  double z = 0.8 * random_get_uniform() - 0.4; // "z" is standard "y" coordinate here...

  robot_trans0[0] = x;
  robot_trans0[1] = 0;
  robot_trans0[2] = z;

  if (is_in_circle(x1, z1, radius, x, z) || is_in_circle(x2, z2, radius, x, z)) {
    robot_trans0[0] = 0;
    robot_trans0[2] = 0;
  }

  robot_rot0[0] = 0;
  robot_rot0[1] = 1;
  robot_rot0[2] = 0;
  robot_rot0[3] = 360 * random_get_uniform() - 180;

  wb_supervisor_field_set_sf_vec3f(robot_translation, robot_trans0);
  wb_supervisor_field_set_sf_rotation(robot_rotation, robot_rot0);

  const char* message = "RESET";
  wb_emitter_send(emitter, message, strlen(message) + 1);

  // evaluation genotype during one minute
  run_seconds(EVOLVE_TIME);

  // measure fitness
  double fitness = measure_fitness();
  printf("Supervisor measure fitness: %f\n", fitness);
  genotype_set_fitness(genotype, fitness);

  printf("fitness: %g\n", fitness);
}

void run_optimization() {
  wb_robot_keyboard_disable();

  printf("---\n");
  printf("starting GA optimization ...\n");
  printf("population size is %d, genome size is %d\n", POPULATION_SIZE, GENOTYPE_SIZE);

  int i, j;
  for  (i = 0; i < NUM_GENERATIONS; i++) {
    for (j = 0; j < POPULATION_SIZE; j++) {
      printf("generation: %d, genotype: %d\n", i, j);

      // evaluate genotype
      Genotype genotype = population_get_genotype(population, j);
      evaluate_genotype(genotype);
    }

    double best_fitness = genotype_get_fitness(population_get_fittest(population));
    double average_fitness = population_compute_average_fitness(population);

    // display results
    plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness: %g\n", best_fitness);
    printf("average fitness: %g\n", average_fitness);

    // reproduce (but not after the last generation)
    if (i < NUM_GENERATIONS - 1)
      population_reproduce(population);
  }

  printf("GA optimization terminated.\n");

  // save fittest individual
  Genotype fittest = population_get_fittest(population);
  FILE *outfile = fopen(FILE_NAME, "w");
  if (outfile) {
    genotype_fwrite(fittest, outfile);
    fclose(outfile);
    printf("wrote best genotype into %s\n", FILE_NAME);
  }
  else
    printf("unable to write %s\n", FILE_NAME);

  population_destroy(population);
}

// show demo of the fittest individual
void run_demo() {
  wb_robot_keyboard_enable(time_step);

  printf("running demo of best individual ...\n");
  printf("select the 3D window and push the 'O' key\n");
  printf("to start genetic algorithm optimization\n");

  FILE *infile = fopen(GENOTYPE_FILE_NAME, "r");
  if (! infile) {
    printf("unable to read %s\n", GENOTYPE_FILE_NAME);
    return;
  }

  Genotype genotype = genotype_create();
  genotype_fread(genotype, infile);
  fclose(infile);


  while (demo) {
     evaluate_genotype(genotype);
  }
}

int main(int argc, const char *argv[]) {

  // initialize Webots
  wb_robot_init();

  // get simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();

  // the emitter to send genotype to robot
  emitter = wb_robot_get_device("emitter");

  // to display the fitness evolution
  display = wb_robot_get_device("display");
  display_width = wb_display_get_width(display);
  display_height = wb_display_get_height(display);
  wb_display_draw_text(display, "fitness", 2, 2);

  // initial population
  printf("GENOTYP SIZE %d INPU_SIZE %d EXP %d HIDDEN %d", GENOTYPE_SIZE, NUM_INPUT, ((NUM_INPUT*NUM_HIDDEN)), NUM_HIDDEN);
  population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);

  // find robot node and store initial position and orientation
  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  robot_translation = wb_supervisor_node_get_field(robot, "translation");
  robot_rotation = wb_supervisor_node_get_field(robot, "rotation");

  memcpy(robot_trans0, wb_supervisor_field_get_sf_vec3f(robot_translation), sizeof(robot_trans0));
  memcpy(robot_rot0, wb_supervisor_field_get_sf_rotation(robot_rotation), sizeof(robot_rot0));

  // run GA optimization
  run_optimization();

  // cleanup Webots
  wb_robot_cleanup();
  return 0;  // ignored
}
