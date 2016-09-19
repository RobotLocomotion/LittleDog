drake_add_matlab_test(NAME examples/LittleDog/LittleDog.runPDhome
  COMMAND LittleDog.runPDhome)
drake_add_matlab_test(NAME examples/LittleDog/LittleDog.runPassive
  COMMAND LittleDog.runPassive)
drake_add_matlab_test(NAME examples/LittleDog/gaitOptimization
  COMMAND gaitOptimization REQUIRES lcm libbot)
