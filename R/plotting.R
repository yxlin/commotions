## From commotions.R  =========================================
##' Plot speed trajectory mix case
##' 
##' Plot speed trajectory
##' 
##' @param obj an output from test_Run
##' 
##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_speed <- function(obj, fn = NULL) 
{
  aname <- names(obj$Parameters)
  nt <- nrow(obj$times)
  d2 <- data.frame(TIME = rep(obj$times[,1], 2), 
                   S    = c(obj$speeds[1,], obj$speeds[2,]),
                   A    = rep(aname, each = nt))
  p1 <- ggplot(d2) +
    geom_line(aes(x = TIME, y = S, color = A ), size=1.5) +
    xlab("Time (s)") + ylab("m/s") +
    theme(legend.position="top")
  
  if (is.null(fn)) {
    invisible( print(p1) )
  } else {
    png(fn)
    invisible( print(p1) )
    dev.off()
  }
}

##' Plot acceleration trajectory mix case
##' 
##' Plot acceleration trajectory
##' 
##' @param obj an output from test_Run
##' 
##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_acc <- function(obj, fn = NULL) 
{
  aname <- names(obj$Parameters)
  nt <- nrow(obj$times)
  d2 <- data.frame(TIME = rep(obj$times[,1], 2), 
                   S    = c(obj$accelerations[1,1:nt], 
                            obj$accelerations[2,1:nt]),
                   A    = rep(aname, each = nt))
  p1 <- ggplot(d2) +
    geom_line(aes(x = TIME, y = S, color = A ), size=1.5) +
    xlab("Time (s)") + ylab("a (m/s^2)") +
    theme(legend.position="top")
  if (is.null(fn)) {
    invisible( print(p1) )
  } else {
    png(fn)
    invisible( print(p1) )
    dev.off()
  }
}

##' Plot travelling trajectory
##'
##' Trajectory of two pedestrian agents using Agent Class
##' 
##' @param obj an output from test_Run
##' @param wall draw a wall segment
##' @param fixed_x x limit for the trajectory 
##' @param fixed_y y limit for the trajectory 
##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_trajectory_AC <- function(obj, wall = NULL, fixed_x = FALSE, 
                               fixed_y = FALSE, fn = NULL)
{
  delta_x_traj <- obj[[2]]$pos[1,] - obj[[1]]$pos[1,]
  delta_y_traj <- obj[[2]]$pos[2,] - obj[[1]]$pos[2,]
  distance_traj <- sqrt(delta_x_traj^2+ delta_y_traj^2)
  is_colliding_traj <- distance_traj < obj[[1]]$collision_d[1]
  
  nc <- sum(is_colliding_traj)
  na <- length(obj)
  nt <- nrow(obj[[1]]$time)
  
  if(fixed_x[1]){
    xlim <- fixed_x
  } else {
    xlim <- range(range(obj[[1]]$pos[1,], obj[[2]]$pos[1,], 
                        obj[[1]]$goal[1,1], obj[[2]]$goal[1,1])) 
  }
  if(fixed_y[1]){
    ylim <- fixed_y
  } else {
    ylim <- range(range(obj[[1]]$pos[2,], obj[[2]]$pos[2,], 
                        obj[[1]]$goal[2,1], obj[[2]]$goal[2,1])) 
  }
  
  
  ## Data for figure 1
  d <- data.frame(X = c(obj[[1]]$pos[1,], obj[[2]]$pos[1,]),
                  Y = c(obj[[1]]$pos[2,], obj[[2]]$pos[2,]), 
                  A = rep(c("A1", "A2"), each = nt))
  
  dc <- data.frame(X = c(obj[[1]]$pos[1, is_colliding_traj], 
                         obj[[2]]$pos[1, is_colliding_traj]),
                   Y = c(obj[[1]]$pos[2, is_colliding_traj], 
                         obj[[2]]$pos[2, is_colliding_traj]),
                   A = rep(c("A1", "A2"), each = nc)) 
  dp <- data.frame(X = c(obj[[1]]$start[1], obj[[2]]$start[1],
                         obj[[1]]$goal[1],  obj[[2]]$goal[1]),
                   
                   Y = c(obj[[1]]$start[2], obj[[2]]$start[2],
                         obj[[1]]$goal[2],  obj[[2]]$goal[2]),
                   A = rep(c("A1", "A2"), 2),
                   TYPE = rep(c("Start", "Goal"), each=2))
  
  
  ## Data for figure 2
  d2 <- data.frame(TIME = rep(obj[[1]]$times[,1], 5), 
                   S = c(obj[[1]]$speeds[,1],
                         obj[[1]]$angles[,1] * 180 / pi,
                         obj[[2]]$speeds[,1],
                         obj[[2]]$angles[,1] * 180 / pi,
                         distance_traj),
                   A  = c( rep(c('A1', 'A2'), each = 2*nt), rep('A2-A1', nt)),
                   gp = c( rep( rep(c('speed', 'angle'), each = nt), 2), 
                           rep('distance', nt)))
  ddc <- data.frame(X = obj[[1]]$times[is_colliding_traj], 
                    Y = distance_traj[is_colliding_traj],
                    gp = rep('distance', nc))
  
  ## Plot figure 1 and 2
  p0 <- ggplot(d, aes(x=X, y=Y, color=A)) +
    geom_point() +
    geom_point(data = dc, aes(x=X, y=Y), colour="blue") +
    geom_point(data = dp, aes(x=X, y=Y, color = A, shape=TYPE), size=3) +
    xlab("X (m)") + ylab("Y (m)") +
    coord_cartesian(xlim=xlim, ylim=ylim) +
    theme(legend.position="top")
  p0
  
  if (!is.null(wall)) {
    p0 <- p0 + geom_segment(x = wall[1], y = wall[2], xend = wall[3], 
                            yend = wall[4], color = "black", size =2) 
  }
  
  p1 <- ggplot(d2) +
    geom_line(aes(x = TIME, y = S, color = A ), size=1.5) +
    geom_point(data = ddc, aes(x=X, y=Y), colour="blue") +
    facet_grid(gp ~ ., scales = 'free_y') + 
    xlab("Time (s)") +
    theme(legend.position="none")
  
  if (is.null(fn)) {
    invisible(grid.arrange(p0, p1, ncol=1))
  } else {
    png(fn)
    p2 <- grid.arrange(p0, p1, ncol=1)
    dev.off()
  }
  
}

##' Plot travelling trajectory
##'
##' Trajectory of a pedestrian agent and a vehicle agent
##' 
##' @param obj an output from test_Run
##' @param wall draw a wall segment
##' @param fixed_x x limit for the trajectory 
##' @param fixed_y y limit for the trajectory 
##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_V2V_1Agent <- 
  function(obj, fixed_x = FALSE, fixed_y = FALSE, fn = NULL)
  {
    nt <- nrow(obj$times)
    
    d <- data.frame(X = c(obj$x[1,]), Y = c(obj$y[1,]),
                    A = rep(c("A0"), each = nt)) 
    
    dp <- data.frame(X = c(obj$start[1,1], obj$goal[1,1], obstacles[,1]),
                     Y = c(obj$start[1,2], obj$goal[1,2], obstacles[,2]),
                     A = rep(c("A0"), 4),
                     TYPE = c("Start", "Goal", "Obstacle", "Obstacle"))
    
    d2 <- data.frame(TIME = rep(obj$times[,1], 5), 
                     S = c(obj$accelerations[1,1:nt],
                           obj$yaw_rate_history,
                           obj$speeds[1,], 
                           obj$sw_angles[1,1:nt] * 180 / pi,
                           obj$angles[1,] * 180 / pi),
                     A  = rep('A0', 5*nt),
                     gp = rep(c('acceleration', 'yaw rate', 'speed', 
                                'steering angle', 'heading angle'), each = nt))
    
    
    if(fixed_x[1]){
      xlim <- fixed_x
    } else {
      xlim <- range(range(obj$x, obj$goal[1,1], obj$obstacles[,1])) 
    }
    if(fixed_y[1]){
      ylim <- fixed_y
    } else {
      ylim <- range(range(obj$y, obj$goal[1,2], obj$obstacles[,2])) 
    }
    
    
    p0 <- ggplot(d, aes(x=X, y=Y, color=A)) +
      geom_point() +
      geom_point(data = dp, aes(x=X, y=Y, color = A, shape=TYPE), size=3) +
      xlab("X (m)") + ylab("Y (m)") +
      coord_cartesian(xlim=xlim, ylim=ylim) +
      theme(legend.position="top")
    
    p1 <- ggplot(d2) +
      geom_line(aes(x = TIME, y = S, color = A ), size=1.5) +
      facet_grid(gp ~ ., scales = 'free_y') + 
      xlab("Time (s)") +
      theme(legend.position="none")
    
    if (is.null(fn)) {
      invisible(grid.arrange(p0, p1, ncol=1))
    } else {
      png(fn)
      p2 <- grid.arrange(p0, p1, ncol=1)
      dev.off()
    }
  }

##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_V2V_2Agent <- 
  function(obj, fixed_x = FALSE, fixed_y = FALSE, fn = NULL)
  {
    nt <- nrow(obj$times)
    
    
    if(fixed_x[1]){
      xlim <- fixed_x
    } else {
      xlim <- range(range(obj$x, obj$goal[,1])) 
    }
    if(fixed_y[1]){
      ylim <- fixed_y
    } else {
      ylim <- range(range(obj$y, obj$goal[,2])) 
    }
    
    d <- data.frame(X = c(obj$x[1,], obj$x[2,]), Y = c(obj$y[1,], obj$y[2,]),
                    A = rep(c("A0", "A1"), each = nt)) 
    
    dp <- data.frame(X = c(obj$start[,1], obj$goal[,1]),
                     Y = c(obj$start[,2], obj$goal[,2]),
                     A = rep(c("A0", "A1"), 2),
                     TYPE = rep(c("Start", "Goal"), each = 2))
    
    d2 <- data.frame(TIME = rep(obj$times[1:nt,1], 6), 
                     S = c(obj$accelerations[1,1:nt], obj$accelerations[2,1:nt],
                           obj$yaw_rate_history[1,1:nt], obj$yaw_rate_history[2,1:nt],
                           obj$sw_angles[1,1:nt] * 180 / pi,
                           obj$sw_angles[2,1:nt] * 180 / pi),
                     A  = rep( rep(c('A0','A1'), each=nt), 3 ),
                     gp = rep(c('acceleration', 'yaw rate', 'steering angle'), each = 2*nt))
    
    p0 <- ggplot(d, aes(x=X, y=Y, color=A)) +
      geom_point() +
      geom_point(data = dp, aes(x=X, y=Y, color = A, shape=TYPE), size=3) +
      xlab("X (m)") + ylab("Y (m)") +
      coord_cartesian(xlim=xlim, ylim=ylim) +
      theme(legend.position="top")
    p1 <- ggplot(d2) +
      geom_line(aes(x = TIME, y = S, color = A ), size=1.5) +
      facet_grid(gp ~ ., scales = 'free_y') + 
      xlab("Time (s)") +
      theme(legend.position="none")
    
    if (is.null(fn)) {
      invisible(grid.arrange(p0, p1, ncol=1))
    } else {
      png(fn)
      p2 <- grid.arrange(p0, p1, ncol=1)
      dev.off()
    }
    
  }

##' @export
plot_P2V_collision <- function(obj) {
  
  nt <- nrow(obj$times)
  
  par(mfrow = (c(1, 2)))
  plot(x = obj$times[c(1, nt),1], y = c(0, 0), type = "l", lty = "dashed",
       ylim = c(-10, 10), xlab = "Time (s)", ylab = expression('d'[CP]*'(m)'))  
  lines(obj$times[,1], -obj$y[1, ], col = "lightblue")
  lines(obj$times[,1], obj$x[2, ], col = "red")
  
  delta_x_traj <- obj$x[2,] - obj$x[1,]
  delta_y_traj <- obj$y[2,] - obj$y[1,]
  distance_traj <- sqrt(delta_x_traj^2+ delta_y_traj^2)
  is_colliding_traj <- distance_traj < obj$collision_distance[1]
  
  plot(x = obj$times[c(1, nt),1], y = obj$collision_distance[1] * c(1, 1),
       type = "l", lty = "dashed", ylim = c(-1, 10), lwd = 3, col = "red",
       xlab = 'Time (s)', ylab = "d(m)")
  lines(obj$times[,1], distance_traj, lwd = 2)
  lines(obj$times[is_colliding_traj, 1], distance_traj[is_colliding_traj], 
        lwd = 2, col = "lightblue")
  par(mfrow = (c(1, 1)))
  
}  

##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_one <- function(x, fixed_x = FALSE, fixed_y = FALSE,
                     collision_distance = 1, fn = NULL)
{
  ## x <- res
  ## fixed_x <- fixed_y <- F
  na <- length(x)
  obj <- x[[1]]
  nt  <- nrow(obj$time)
  has_obstacles <- !any(is.na(obj$obstacles))
  if (fixed_x[1]) { xlim <- fixed_x } else {xlim <- range(obj$pos[1,])}
  if (fixed_y[1]) { ylim <- fixed_y } else {ylim <- range(obj$pos[2,])}
  
  ## Figure 1
  d <- data.frame(X = c(obj$pos[1,]), Y = c(obj$pos[2,]),
                  A = rep("A0", nt))
  
  dp <- data.frame(X = c(obj$pos[1,1], obj$goal[1,1]),
                   Y = c(obj$pos[2,1], obj$goal[2,1]), 
                   A = rep(c("A0"), 2),
                   TYPE = c("Start", "Goal"))
  ## Figure 2
  d2 <- data.frame(TIME = rep(obj$times[,1], 2), 
                   S = c(obj$speeds[,1], 
                         obj$angles[,1] * 180 / pi),
                   A  = rep(c('A0'), 2*nt),
                   gp = rep(c('speed (m/s)', 'angle (deg)'), each = nt))
  
  p0 <- ggplot(d, aes(x=X, y=Y, color=A)) +
    geom_point() +
    geom_point(data = dp, aes(x=X, y=Y, shape=TYPE), size=3) +
    xlab("X (m)") + ylab("Y (m)") +
    # coord_cartesian(xlim=xlim, ylim=ylim) +
    theme(legend.position="top")
  
  if (has_obstacles)
  {
    do <- data.frame(X = obj$obstacles[1,], Y=obj$obstacles[2,])
    p0 <- p0 + geom_point(data = do, aes(x=X, y=Y), colour="red", shape = 15, 
                          size = 5)
  }
  
  p1 <- ggplot(d2) +
    geom_line(aes(x = TIME, y = S), size=1.5) +
    facet_grid(gp ~ ., scales = 'free_y') + 
    xlab("Time (s)") +
    theme(legend.position="none")
  
  
  
  ## Figure 2 & 3
  if (is.null(fn)) {
    invisible(  grid.arrange(p0, p1, ncol=1))
  } else {
    png(fn)
    p2 <- grid.arrange(p0, p1, ncol=1)
    dev.off()
  }
}


##' Plot Tsee trajectories
##'
##' Find first possible collision point  
##' 
##' @param obj a list storing trajectory (x, y), speed, heading angles, and 
##' time stamps from a simulation run 
##' @param wall the wall segment
##' 
##' @export
plot_tsee <- function(obj, wall = NULL) 
{
  # obj <- traj
  nt   <- nrow(obj$times)
  dt   <- obj$times[2,1] - obj$times[1,1]; 
  blocked <- rep(FALSE, nt)
  
  if (!is.null(wall))
  {
    for(i in 1:nt)
    {
      segment0 <- c(obj$x[1, i], obj$y[1, i], obj$x[2, i], obj$y[2, i])
      if(is_cross(segment0 = segment0, segment1 = wall)) blocked[i] <- TRUE
    }
    
    i_tsee <- which.min(blocked)
    tsee   <- obj$times[i_tsee, 1]
    tsee_lab <- bquote("T"[see]~.(paste0('= ', tsee, ' s')))
  }
  
  d0 <- data.frame(X = as.vector(obj$x[1:2,]), 
                   Y = as.vector(obj$y[1:2,]), 
                   A = rep(c('A0', 'A1'), nt))
  d1 <- data.frame(X = c(obj$x[1:2, 1], obj$goal[1,]),
                   Y = c(obj$y[1:2, 1], obj$goal[2,]),
                   A = rep(c('A0', 'A1'), 2),
                   G = rep(c('Start', 'Goal'), each=2)) 
  d2 <- data.frame(xstart=obj$x[1,], xend=obj$x[2,], ystart=obj$y[1,], 
                   yend=obj$y[2,], B = blocked, A = 'AX')
  
  p0 <- ggplot(data=d0) +
    geom_line(aes(x=X, y=Y, color=A)) +
    geom_point(data=d1, aes(x=X, y=Y, shape=G, color=A), size = 5) +
    geom_segment(data = d2[d2$B,],
                 aes(x=xstart, xend=xend, y=ystart, yend=yend), 
                 linetype="dotted",
                 arrow = arrow(length = unit(0.5, "cm"))) +
    theme(legend.position="top")
  
  if (!is.null(wall) )
  {
    p0 <- p0 + geom_segment(x = wall[1], y = wall[2], xend = wall[3],
                            yend = wall[4], color = "black", size =2) +
      annotate("text",
               x = obj$x[1, i_tsee],
               y = obj$y[1, i_tsee],
               label = paste0("Tsee = ", tsee),
               size = 5, colour = "red")
  }
  print(p0)
  invisible(p0)
}



##' Plot time to collision trajectory one agent
##'
##' Trajectory of a pedestrian agent and a vehicle agent. Note when using
##' fixed_y, the first limit should set a small value, instead of 0.
##' 
##' @export
plot_ttc <- function(obj, fixed_x = FALSE, fixed_y = FALSE) {
  
  if(fixed_x[1]) {
    xlim <- fixed_x
  } else {
    xlim <- range(obj$times)
  }
  if(fixed_y[1]) {
    ylim <- fixed_y
  } else {
    ylim <- range( obj$ttc_history[is.finite(obj$ttc_history)] )
  }
  
  plot(obj$times[,1], obj$ttc_history[,1], xlim = xlim, ylim = ylim,
       type = "l", lwd = 2, col = "orange",
       xlab = 'Time (s)', ylab = 'TTC (s)')
  lines(obj$times[,1], obj$ttc_history[,2], lwd = 2, col = "lightblue")
  
}

##' @export
plot_init <- function(start, goal, obstacles = NULL)
{
  na <- ncol(start)
  xlim <- range(start[1,], goal[1,])
  ylim <- range(start[2,], goal[2,])
  
  plot(start[1,1], start[2,1], cex =2, xlim = xlim, ylim = ylim,
       xlab = "x (m)", ylab = "y (m)")
  points(goal[1,1], goal[2,1], cex =2, pch = 16)
  
  if (na == 2) 
  {
    points(start[1,2], start[2,2], cex =2, pch=2)
    points(goal[1,2], goal[2,2], cex =2, pch = 17)
    abline(v=0, lty = "dotted")
    abline(h=0, lty = "dotted")
  }
  
  if(!is.null(obstacles)) 
  {
    points(obstacles[1,1], obstacles[2,1], cex =2, pch = 13)
  }
  
}

## Trajectory plots ===========================================================
##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_1traj <- function(obj, fixed_x = FALSE, fixed_y = FALSE, fn = NULL)
{
  na <- nrow(obj$x)
  nt  <- nrow(obj$time)
  has_obstacles <- !any( is.na( obj$obstacles ) )
  
  if (fixed_x[1]) { xlim <- fixed_x } else {xlim <- range(obj$x, obj$goal[1,])}
  if (fixed_y[1]) { ylim <- fixed_y } else {ylim <- range(obj$y, obj$goal[2,])}
  
  ## Figure 1
  d <- data.frame(X = c(obj$x[1,]), Y = c(obj$y[1,]),
                  A = rep("A0", nt)) 
  
  dp <- data.frame(X = c(obj$x[1,1], obj$goal[1,1], obj$obstacles[1,1]),
                   Y = c(obj$y[1,1], obj$goal[2,1], obj$obstacles[2,1]), 
                   A = rep(c("A0"), 3),
                   P = c("Start", "Goal", "Obstacle"))

  p0 <- ggplot(d, aes(x=X, y=Y)) +
    geom_point() +
    geom_point(data = dp, aes(x=X, y=Y, shape=P), size=3) +
    xlab("X (m)") + ylab("Y (m)") +
    coord_cartesian(xlim=xlim, ylim=ylim) +
    theme_minimal(base_size = 16) +
    theme(legend.position="top")
  print(p0)
}

##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_1kinematics <- function(obj, fixed_x = FALSE, fixed_y = FALSE, fn = NULL)
{
  na <- nrow(obj$x)
  nt  <- nrow(obj$time)
  has_obstacles <- !any( is.na( obj$obstacles ) )
  
  ## Figure 2
  d2 <- data.frame(TIME = rep(obj$times[,1], 2), 
                   S = c(obj$speeds[1,], 
                         obj$angles[1,] * 180 / pi),
                   A  = rep(c('A0'), 2*nt),
                   gp = rep(c('speed (m/s)', 'angle (deg)'), each = nt))

  p0 <- ggplot(d2) +
    geom_line(aes(x = TIME, y = S), size=1.5) +
    facet_grid(gp ~ ., scales = 'free_y', switch = "y") + 
    xlab("Time (s)") + ylab("") +
    theme_minimal(base_size = 16) +
    theme(legend.position="none",
          strip.placement = "outside")
  print(p0)
}

##' Plot travelling trajectory
##' 
##' Trajectory of two pedestrian agents using Simulation and Kinematics Classes
##'
##' @param obj an output from test_Run
##' @param wall draw a wall segment
##' @param fixed_x x limit for the trajectory 
##' @param fixed_y y limit for the trajectory 
##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_2traj <- function(obj, wall = NULL, fixed_x = FALSE, 
                       fixed_y = FALSE, collision_distance = 1, fn = NULL)
{
  obj <- traj
  fixed_x <- F
  fixed_y <- F
  c_traj   <- is_collision(obj)[[1]]  ## collision_trajectory
  distance <- is_collision(obj)[[2]]  ## collision_trajectory
  aname <- names(obj$Parameters)
  nc <- sum(c_traj)
  na <- nrow(obj$x)
  nt <- nrow(obj$time)
  if(fixed_x[1]){
    xlim <- fixed_x
  } else {
    xlim <- range(range(obj$x, obj$goal[,1])) 
  }
  if(fixed_y[1]){
    ylim <- fixed_y
  } else {
    ylim <- range(range(obj$y, obj$goal[,2])) 
  }
  

  ## Figure 1
  d <- data.frame(X = c(obj$x[1,], obj$x[2,]), Y = c(obj$y[1,], obj$y[2,]),
                  A = rep(aname, each = nt)) 
  dc <- data.frame(X = c(obj$x[1, c_traj], 
                         obj$x[2, c_traj]),
                   Y = c(obj$y[1, c_traj], 
                         obj$y[2, c_traj]),
                   A = rep(aname, each = nc)) 
  
  dp <- data.frame(X = c(obj$x[,1], obj$goal[1,]),
                   Y = c(obj$y[,2], obj$goal[2,]),
                   A = rep(aname, 2),
                   TYPE = rep(c("Start", "Goal"), each=2))
  
  cb <- Manu::get_pal("Tui")
  p0 <- ggplot(d, aes(x=X, y=Y, color=A)) +
    geom_point() +
    geom_point(data = dc, aes(x=X, y=Y), color = cb[6]) +
    geom_point(data = dp, aes(x=X, y=Y, color = A, shape=TYPE), size=3) +
    scale_color_manual(values = cb) +
    xlab("X (m)") + ylab("Y (m)") +
    coord_cartesian(xlim=xlim, ylim=ylim) +
    theme(legend.position="top")
  
  if (!is.null(wall)) {
    p0 <- p0 + geom_segment(x = wall[1], y = wall[2], xend = wall[3], 
                            yend = wall[4], color = "black", size =2) 
  }
  print(p0)
}

##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_2kinematics <- function(obj, wall = NULL, fixed_x = FALSE, 
                       fixed_y = FALSE, collision_distance = 1, fn = NULL)
{
  c_traj   <- is_collision(obj)[[1]]  ## collision_trajectory
  distance <- is_collision(obj)[[2]]  ## collision_trajectory
  aname <- names(obj$Parameters)
  nc <- sum(c_traj)
  na <- nrow(obj$x)
  nt <- nrow(obj$time)
  if(fixed_x[1]){
    xlim <- fixed_x
  } else {
    xlim <- range(range(obj$x, obj$goal[,1])) 
  }
  if(fixed_y[1]){
    ylim <- fixed_y
  } else {
    ylim <- range(range(obj$y, obj$goal[,2])) 
  }
  
  ## Figure 2
  d2 <- data.frame(TIME = rep(obj$times[,1], 5), 
                   S = c(obj$speeds[1,], 
                         obj$angles[1,] * 180 / pi,
                         obj$speeds[2,],
                         obj$angles[2,] * 180 / pi,
                         distance),
                   A  = c( rep(aname, each = 2*nt), rep('A2-A1', nt)),
                   gp = c( rep( rep(c('speed', 'angle'), each = nt), 2), 
                           rep('distance', nt)))
  
  ddc <- data.frame(X = obj$times[c_traj], 
                    Y = distance[c_traj],
                    gp = rep('distance', nc))
  
  cb <- Manu::get_pal("Tui")
  p1 <- ggplot(d2) +
    geom_line(aes(x = TIME, y = S, color = A ), size=1.5) +
    geom_point(data = ddc, aes(x=X, y=Y), colour=cb[6]) +
    scale_color_manual(values = cb) +
    facet_grid(gp ~ ., scales = 'free_y') + 
    xlab("Time (s)") + ylab("") +
    theme(legend.position="top")
  print(p1)
}

##' Plot travelling trajectory
##' 
##' Trajectory of two pedestrian agents using Simulation and Kinematics Classes
##'
##' @param obj an output from test_Run
##' @param fixed_x x limit for the trajectory 
##' @param fixed_y y limit for the trajectory 
##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_trajOnly <- function(obj, fixed_x = FALSE, fixed_y = FALSE,
                          fn = NULL)
{
  ## obj <- res
  ## fixed_x <- fixed_y <- F
  aname <- names(obj$Parameters)
  na <- nrow(obj$x)
  nt <- nrow(obj$time)
  
  if(fixed_x[1]){
    xlim <- fixed_x
  } else {
    xlim <- range(range(obj$x, obj$goal[1,])) 
  }
  if(fixed_y[1]){
    ylim <- fixed_y
  } else {
    ylim <- range(range(obj$y, obj$goal[2,])) 
  }
  
  ## Figure 1
  d <- data.frame(X = c(obj$x[1,], obj$x[2,]), 
                  Y = c(obj$y[1,], obj$y[2,]),
                  A = rep(aname, each = nt)) 
  dp <- data.frame(X = c(obj$x[,1], obj$goal[1,]),
                   Y = c(obj$y[,1], obj$goal[2,]),
                   A = rep(aname, 2),
                   TYPE = rep(c("Start", "Goal"), each=2))
  
  
  p0 <- ggplot(d, aes(x=X, y=Y, color=A)) +
    geom_point() +
    geom_point(data = dp, aes(x=X, y=Y, color = A, shape=TYPE), size=3) +
    xlab("X (m)") + ylab("Y (m)") +
    coord_cartesian(xlim=xlim, ylim=ylim) +
    theme(legend.position="top")
  
  if (is.null(fn)) {
    invisible(print(p0))
  } else {
    png(fn)
    invisible(print(p0))
    dev.off()
  }
}

##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_2traj_no_angle <- function(obj, wall = NULL, fixed_x = FALSE, 
                                fixed_y = FALSE, collision_distance = 1,
                                fn = NULL,
                                col = "Set1")
{
  # cbPalette <- c("#999999", "#E69F00", "#56B4E9", "#009E73", "#F0E442", "#0072B2", "#D55E00", "#CC79A7")
  # cbPalette <- gray.colors(3)
  cbPalette <- RColorBrewer::brewer.pal(5, col)
  
  # obj <- traj
  nt   <- nrow(obj$times)
  dt   <- obj$times[2,1] - obj$times[1,1]; 
  blocked <- rep(FALSE, nt)
  aname <- names(obj$Parameters)
  
  if (!is.null(wall))
  {
    for(i in 1:nt)
    {
      segment0 <- c(obj$x[1, i], obj$y[1, i], obj$x[2, i], obj$y[2, i])
      if(is_cross(segment0 = segment0, segment1 = wall)) blocked[i] <- TRUE
    }
    
    i_tsee <- which.min(blocked)
    tsee   <- obj$times[i_tsee, 1]
    tsee_lab <- bquote("T"[see]~.(paste0('= ', tsee, ' s')))
  }
  
  d0 <- data.frame(X = as.vector(obj$x[1:2,]), 
                   Y = as.vector(obj$y[1:2,]), 
                   A = rep(aname, nt))
  d1 <- data.frame(X = c(obj$x[1:2, 1], obj$goal[1,]),
                   Y = c(obj$y[1:2, 1], obj$goal[2,]),
                   A = rep(aname, 2),
                   G = rep(c('Start', 'Goal'), each=2)) 
  d2 <- data.frame(xstart=obj$x[1,], xend=obj$x[2,], ystart=obj$y[1,], 
                   yend=obj$y[2,], B = blocked, A = 'PX')
  
  p0 <- ggplot(data=d0) +
    geom_line(aes(x=X, y=Y, color=A), size=1.5) +
    geom_point(data=d1, aes(x=X, y=Y, shape=G, color=A), size = 5) +
    scale_colour_manual(values=cbPalette) +
    geom_segment(data = d2[d2$B,],
                 aes(x=xstart, xend=xend, y=ystart, yend=yend), 
                 linetype="dotted",
                 arrow = arrow(length = unit(0.5, "cm"))) +
    theme_bw(base_size = 16) +
    theme(legend.position="top",
          legend.title = element_blank())
  
  if (!is.null(wall) )
  {
    p0 <- p0 + geom_segment(x = wall[1], y = wall[2], xend = wall[3],
                            yend = wall[4], color = "black", size =2) +
      annotate("text",
               x = obj$x[1, i_tsee],
               y = obj$y[1, i_tsee],
               label = paste0("Tsee = ", tsee, " (s)"),
               size = 5)
  }
  
  c_traj   <- is_collision(obj)[[1]]  ## collision_trajectory
  distance <- is_collision(obj)[[2]]  ## collision_trajectory
  nc <- sum(c_traj)
  na <- nrow(obj$x)
  
  d2 <- data.frame(TIME = rep(obj$times[,1], 5), 
                   S  = c(obj$speeds[1,], obj$speeds[2,], 
                          obj$accelerations[1,1:nt], obj$accelerations[2,1:nt],
                          distance),
                   A  = c( rep( rep(aname, each = nt), 2), rep('P1-P0', nt)),
                   gp = c( rep('S (m/s)', 2*nt), 
                           rep('A (m/s^2)', 2*nt), 
                           rep('D (m)', nt)))
  
  ddc <- data.frame(X = obj$times[c_traj], 
                    Y = distance[c_traj],
                    gp = rep('D (m)', nc))
  
  p1 <- ggplot(d2) +
    geom_line(aes(x = TIME, y = S, color = A ), size=1.5) +
    scale_colour_manual(values=cbPalette) +
    geom_vline(xintercept = tsee, linetype = "dotted") +
    geom_point(data = ddc, aes(x=X, y=Y), colour="blue") +
    facet_grid(gp ~ ., scales = 'free_y') + 
    xlab("Time (s)") + ylab("") +
    theme_bw(base_size = 16) +
    theme(legend.position="top",
          legend.title = element_blank())
  
  invisible(grid.arrange(p0, p1, ncol=2))
  
}

##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_2traj_no_angle_no_tsee <- function(obj, tta, fixed_x = FALSE, 
                                fixed_y = FALSE, col = "Set1")
{
  # obj <- traj_VYield
  # col <- "Set1"
  cbPalette <- RColorBrewer::brewer.pal(5, col)
  
  nt   <- nrow(obj$times)
  dt   <- obj$times[2,1] - obj$times[1,1]; 
  blocked <- rep(FALSE, nt)
  aname <- names(obj$Parameters)
  

  d0 <- data.frame(X = as.vector(obj$x[1:2,]), 
                   Y = as.vector(obj$y[1:2,]), 
                   A = rep(aname, nt))
  d1 <- data.frame(X = c(obj$x[1:2, 1]),
                   Y = c(obj$y[1:2, 1]),
                   A = aname,
                   G = rep(c('Start'), each=2)) 
  d2 <- data.frame(xstart=obj$x[1,], xend=obj$x[2,], ystart=obj$y[1,], 
                   yend=obj$y[2,], B = blocked, A = 'PX')
  
  p0 <- ggplot(data=d0) +
    geom_line(aes(x=X, y=Y, color=A), size=1.5) +
    geom_point(data=d1, aes(x=X, y=Y, shape=G, color=A), size = 5) +
    scale_colour_manual(values=cbPalette) +
    geom_segment(data = d2[d2$B,],
                 aes(x=xstart, xend=xend, y=ystart, yend=yend), 
                 linetype="dotted",
                 arrow = arrow(length = unit(0.5, "cm"))) +
    annotate("text",
             x = -50,
             y = 2.5,
             label = paste0("TTAi = ", round(tta, 2)),
             size = 5, colour = "red") +

    theme_bw(base_size = 16) +
    theme(legend.position="top",
          legend.title = element_blank())
  ## p0  
  

  dD <- getD_df2(obj)
  p2 <- ggplot(data=dD) +
    geom_line(aes(x=Time, y=D, color=A, linetype = gp), size=1.2) +
    scale_colour_manual(values=cbPalette) +
    facet_grid(type~., scales = "free_y") +
    ylab("")+
    theme_bw(base_size = 16) +
    theme(legend.position = "top",
          legend.title = element_blank(),
          legend.key.size = grid::unit(.5, "cm"),
          legend.box = "horizontal")
  

  invisible(grid.arrange(p0, p2, ncol=2))
  
}

##' @export
##' @import ggplot2
##' @importFrom gridExtra grid.arrange 
plot_distance <- function(obj, crossing = c(0, 0))
{
  get_distance <- function(coordinate, midpoint = c(0,0))
  {
    sqrt ( sum( (coordinate - midpoint)^2 ) )
  }
  
  P0 <- obj$x[1,] > crossing[1] & obj$y[1,] > crossing[2]
  P1 <- obj$x[2,] > crossing[1] & obj$y[2,] < crossing[2]
  if( sum(obj$x[1,] > crossing[1]) != sum(obj$y[1,] > crossing[2]) ) 
  {
    stop("P0 problem")
  }
  if (sum(obj$x[2,] > crossing[1]) != sum(obj$y[2,] < crossing[2]) ) 
  {
    stop("P1 problem")
  }
  
  dP0 <- cbind(obj$x[1,], obj$y[1,], P0)
  dP1 <- cbind(obj$x[2,], obj$y[2,], P1)
  
  nt <- nrow(obj$times)
  tmp0 <- numeric(nt)
  tmp1 <- numeric(nt)
  for(i in 1:nt)
  {
    tmp0[i] <- get_distance(dP0[i, 1:2])
    tmp1[i] <- get_distance(dP1[i, 1:2])
  }
  
  n0 <- length(tmp0[dP0[,3]==0])
  n1 <- length(tmp1[dP1[,3]==0])
  
  n2 <- length(tmp0[dP0[,3]==1])
  n3 <- length(tmp1[dP1[,3]==1])
  cbPalette <- RColorBrewer::brewer.pal(5, "Set2")
  
  plot(obj$times[1:n0, 1], tmp0[dP0[,3]==0], xlim=c(0, 30), 
       xlab = "Time (s)", ylab="D to crossing", 
       type = "l", lwd =3, col=cbPalette[1])
  lines(obj$times[1:n1, 1], tmp1[dP1[,3]==0], lwd =3, col=cbPalette[2])
  
  lines(obj$times[(n0+1):nt, 1], tmp0[dP0[,3]==1], lwd=3, col=cbPalette[1],
        lty = "dashed")
  lines(obj$times[(n1+1):nt, 1], tmp1[dP1[,3]==1], lwd=3, col=cbPalette[2],
        lty = "dashed")
  
  Time0 <- c(obj$times[1:n0, 1], obj$times[(n0+1):nt, 1])
  Time1 <- c(obj$times[1:n1, 1], obj$times[(n1+1):nt, 1])
  D0 <- c(tmp0[dP0[,3]==0], tmp0[dP0[,3]==1])
  D1 <- c(tmp1[dP1[,3]==0], tmp1[dP1[,3]==1])
  A  <- rep( names(obj$Parameters), each = nt)
  gp <- c(rep("Prior", n0), rep("After", length( (n0+1):nt ) ),
          rep("Prior", n1), rep("After", length( (n1+1):nt ) )  )
  
  out <- data.frame(Time = c(Time0, Time1), D=c(D0, D1), A = A,
                    gp = gp)
  return(out)
}

##' @export
getD_df <- function(obj, crossing = c(0, 0))
{
  get_distance <- function(coordinate, midpoint = c(0,0))
  {
    sqrt ( sum( (coordinate - midpoint)^2 ) )
  }
  
  P0 <- obj$x[1,] > crossing[1] & obj$y[1,] > crossing[2]
  P1 <- obj$x[2,] > crossing[1] & obj$y[2,] < crossing[2]
  if( sum(obj$x[1,] > crossing[1]) != sum(obj$y[1,] > crossing[2]) ) 
  {
    stop("P0 problem")
  }
  if (sum(obj$x[2,] > crossing[1]) != sum(obj$y[2,] < crossing[2]) ) 
  {
    stop("P1 problem")
  }
  
  dP0 <- cbind(obj$x[1,], obj$y[1,], P0)
  dP1 <- cbind(obj$x[2,], obj$y[2,], P1)
  
  nt <- nrow(obj$times)
  tmp0 <- numeric(nt)
  tmp1 <- numeric(nt)
  for(i in 1:nt)
  {
    tmp0[i] <- get_distance(dP0[i, 1:2])
    tmp1[i] <- get_distance(dP1[i, 1:2])
  }
  
  n0 <- length(tmp0[dP0[,3]==0])  ## Prior crossing P0
  n1 <- length(tmp1[dP1[,3]==0])  ## P1
  
  n2 <- length(tmp0[dP0[,3]==1])  ## after crossing P0
  n3 <- length(tmp1[dP1[,3]==1])  ## P1
  
  Time0 <- c(obj$times[1:n0, 1], obj$times[(n0+1):nt, 1])
  Time1 <- c(obj$times[1:n1, 1], obj$times[(n1+1):nt, 1])
  D0 <- c(tmp0[dP0[,3]==0], tmp0[dP0[,3]==1])
  D1 <- c(tmp1[dP1[,3]==0], tmp1[dP1[,3]==1])
  A  <- rep( names(obj$Parameters), each = nt)
  gp <- c(rep("Prior", n0), rep("After", length( (n0+1):nt ) ),
          rep("Prior", n1), rep("After", length( (n1+1):nt ) )  )
  
  message("P0: ", n0/10, "P1 ", n1/10)
  out <- data.frame(Time = c(Time0, Time1), D=c(D0, D1), A = A,
                    gp = gp)
  return(out)
}

getD_df2 <- function(obj, crossing = c(0, 0))
{
  # obj <- traj_VYield
  get_distance <- function(coordinate, midpoint = c(0,0))
  {
    sqrt ( sum( (coordinate - midpoint)^2 ) )
  }
  
  P0 <- obj$y[1,] > crossing[2]
  P1 <- obj$x[2,] < crossing[1] 
  
  dP0 <- cbind(obj$x[1,], obj$y[1,], P0)
  dP1 <- cbind(obj$x[2,], obj$y[2,], P1)
  
  nt <- nrow(obj$times)
  tmp0 <- numeric(nt)
  tmp1 <- numeric(nt)
  for(i in 1:nt)
  {
    tmp0[i] <- get_distance(dP0[i, 1:2])
    tmp1[i] <- get_distance(dP1[i, 1:2])
  }
  
  n0 <- length(tmp0[dP0[,3]==0])
  n1 <- length(tmp1[dP1[,3]==0])
  
  n2 <- length(tmp0[dP0[,3]==1])
  n3 <- length(tmp1[dP1[,3]==1])
  
  Time0 <- c(obj$times[1:n0, 1], obj$times[(n0+1):nt, 1])
  Time1 <- c(obj$times[1:n1, 1], obj$times[(n1+1):nt, 1])
  D0 <- c(tmp0[dP0[,3]==0], tmp0[dP0[,3]==1])
  D1 <- c(tmp1[dP1[,3]==0], tmp1[dP1[,3]==1])
  A  <- rep( names(obj$Parameters), each = nt)
  gp <- c(rep("Prior", n0), rep("After", length( (n0+1):nt ) ),
          rep("Prior", n1), rep("After", length( (n1+1):nt ) )  )

  message("P: ", n0/10, " V: ", n1/10)
  
  S0 <- c(obj$speeds[1, 1:n0], obj$speeds[1, (n0+1):nt])
  S1 <- c(obj$speeds[2, 1:n1], obj$speeds[2, (n1+1):nt])
  AC0 <- c(obj$accelerations[1, 1:n0], obj$accelerations[1, (n0+1):nt])
  AC1 <- c(obj$accelerations[2, 1:n1], obj$accelerations[2, (n1+1):nt])
  
  anames <- names(obj$Parameters)
  
  out <- data.frame(Time = c(Time0, Time1, Time0, Time1, Time0, Time1), 
                    D=c(D0, D1, S0, S1, AC0, AC1), 
                    A = c(A, A, A),
                    gp = c(gp, gp, gp),
                    type = c(rep("D", 2*nt),
                               rep(paste0("S-", anames[1]), nt),
                               rep(paste0("S-", anames[2]), nt),
                               rep("A", 2*nt)))
  
  return(out)
}


