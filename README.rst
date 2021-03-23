===========================================
commotions: Modelling Traffic Trajectories 
===========================================

.. sectnum::

.. contents:: Table of contents

.. image:: tests/testthat/Group0_OldClass_P2P/figs/TwoAs_test.png
    :width: 200px
    :height: 100px
    :align: left
    :alt: Two Pedestrians

What commotions does?
~~~~~~~~~~~~~~~~~~~~~~~~~
**commotions** simulates the trajectory of pedestrians, vehicles and their 
interactions. The package is part of the COMMOTIONS project, a research project
studying traffic scenarios where pedestrians and drivers interact.

.. image:: tests/testthat/Group2_P1KL/figs/InitialPos.png 
    :width: 200px
    :height: 100px
    :align: right
    :alt: Two Pedestrians

How the codes are organized 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- src folder includes Agents.cpp, Kinematics.cpp, KinematicState.cpp, 
  main.cpp, Parameters.cpp, SCAgent.cpp, SCSimulation.cpp, Simulation.cpp, 
  unit_test.cpp, and util.cpp 
- inst/include/ folder store header files. Kinematics.hpp, Parameters.hpp, Simulation.hpp, main.cpp 
- R folder common routine functions, storing in includes commotions.R, plotting.R, free_speed.R
- tests/testthat/ folders has the following subfolders: 

    - Group0 tests old OO classes for pedestrian trajectories,  
    - Group1 tests new Agent classes for pedestrian trajectoies,
    - Group2 tests P1KL behaviour
    - Group3 tests old OO classes for vehicle trajectories
    - Group4 tests old OO classes for pedestrian-vehicle interaction
    - Group5 tests plot functions, 
    - Group6 tests P6DY
    - Group9 tests P9CDg, etc. 

Where to getting started
~~~~~~~~~~~~~~~~~~~~~~~~~
The following example simulates a pedestrian's trajectory.

::

   na <- 1
   time <- c(st=0, et=15, dt=.1, pt=.3)
   obstacles <- matrix(c(5, 5), ncol = na)
   colnames(obstacles) <- c("O0")
   rownames(obstacles) <- c('x', 'y')
   parameter <- list(P0 = c(type = 0, kg=1, kc=.3, kdv=.3, ke=.3))
   
   pos  <- matrix(c(0, 0), ncol=na)
   goal <- matrix(c(10, 10), ncol=na)
   colnames(pos) <- colnames(goal) <-  c("A0")
   rownames(pos) <- rownames(goal) <- c('x', 'y')
   
   initial_speed <- NA
   initial_angle <- pi/4.01
   ## The default initial speed is 0, if the user enters an NA.
   ## The initial angle is arc-tangent
   atan2(goal[2] - pos[2], goal[1] - pos[1])
   atan((goal[2] - pos[2])/(goal[1] - pos[1]))
   ## See https://en.wikipedia.org/wiki/Atan2 for arc-tangent explanation, if you
   ## are unfamiliar with it 
   
   so <- matrix( c(-1, .5, 1), ncol=na); so
   ao <- matrix( c(-10, 5, 10), ncol=na); ao
   colnames(so) <- colnames(ao) <- "A0"
   rownames(so) <- rownames(ao) <- c("start", "diff", "end")

   wall <- c(-5, 0, -2.5, 0)
   cd <- 1  ## collision_distance
   sr <- 0  ## steering_ratio
   
   traj <- test_P2P(time, parameter, pos, goal, so, ao, initial_speed, 
                   initial_angle, obstacles, cd, sr, wall)
   tmp <- plot_1traj(traj)


Installation
**********************
To install, the user must have the devtools package, which enables a rapid download
the source codes from the author's github.  

::

   devtools::install_github('yxlin/commotions')


.. image:: tests/testthat/Group0_OldClass_P2P/figs/OneA_test.png
    :width: 200px
    :height: 100px
    :align: center
    :alt: One Pedestrian 

