=====================================================
LAPF: Kinematics Leading Agent Passes First 
=====================================================

.. sectnum::

.. contents:: Table of contents

.. image:: figs/X4_issue.png
    :width: 200px
    :height: 100px
    :align: right
    :alt: X4issue

Simulating Trajectories
~~~~~~~~~~~~~~~~~~~~~~~~~

The first line defines start time = 0 s and end time = 30 s, and then defines
time step = .1, s action time interval = .3 s

::

    time <- c(0, 30, .1, .3)
    pos  <- matrix(c(-5, -5, -5, 5), ncol=2); pos
    goal <- matrix(c(5, 5, 5, -5), ncol=2); goal

The options of speed and heading direction. Define x1, y1 and x2 y2 for the wall

::

    so <- matrix( rep(c(-1, .5, 1), 2), ncol=2)
    ao <- matrix( rep(c(-10, 0, 10), 2), ncol=2)
    wall <- c(-5, 0, -2.5, 0)


List all speed combinations

::

    speed_options <- test_regspace(0, .1, 1)[,1]
    speeds <- get_speed_list(speed_options)  ## 10 speed options
    ## [1] 0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9
    nrow(speeds)  ## 10 x 10 = 100 different speed pairs (for P0 and P1)

Prepare the containers to store scoring results. First set up n = 1000 possible
parameter pairs (i.e., [kc kdv]), storing them in a 1000-element list.

::

    ## n <- 10   ## Test run
    n <- 1000    ## actual run
    parameters <- vector("list", length = n)


Then set up containers to store scoring of different behaviours

::

    P1KL <- P2FA <- P3SD <- P2FD <- matrix(rep(FALSE, n*nrow(speeds)), ncol=n)
    P3SA <- X4P0 <- X4P1 <- matrix(rep(FALSE, n*nrow(speeds)), ncol=n)
    isovershoot0 <- isovershoot1 <- matrix(rep(FALSE, n*nrow(speeds)), ncol=n)
    iscolliding <- matrix(rep(FALSE, n*nrow(speeds)), ncol=n)

    farfrom_reaching_goal0 <- matrix(rep(FALSE, n*nrow(speeds)), ncol=n)
    farfrom_reaching_goal1 <- matrix(rep(FALSE, n*nrow(speeds)), ncol=n)

    alread_reach_free_speed0 <- matrix(rep(FALSE, n*nrow(speeds)), ncol=n)
    alread_reach_free_speed1 <- matrix(rep(FALSE, n*nrow(speeds)), ncol=n)


**cd** stands for collisoin distance, representing the collision distances for
each agent. Legacy code for default steering ratio for the agent 1 and agent 2.

::

    cd <- c(1, 1)
    sr <- c(0, 0)


Use a simple two-layer for loops to step through the parameter pairs and the
speed pairs.

::

    for (i in seq_len(n)) 
    {
        ## Cv = kdv; Ct = ke; k_g and C_t == 1
        ## Two identical agents
        kc <- runif(1, 0, 10)
        kdv <- runif(1, .28, .71)
        parameters[[i]] <- list(P0 = c(type=0, kg=1, kc=kc, kdv=kdv, ke=0),
                            P1 = c(type=0, kg=1, kc=kc, kdv=kdv, ke=0))
        # path <- paste0("tests/testthat/Group2_P1KL/figs/para", i)
        # if (!dir.exists(path)) { dir.create(path) }
  
        for (j in seq_len(nrow(speeds))) 
        {
            init_speed <- speeds[j, ]
            traj <- test_P2P(time, parameters[[i]], pos, goal, so, ao, init_speed,
                         init_angle, obstacles, cd, sr, wall)
            # png(file="figs/X4_issue.png", 800, 600)
            plot_2traj_no_angle(traj, wall=wall, col="Set2")
        }    
    }

Analyses
**********************

