######### Calculate free speeds ----------------------------------------------
##' @export
get_car_free_speed <- function(
  time  = c(0, 15, .1, .5), 
  parameter = list(V = c(type=1, kg=1, kc=.5, kdv=.036, ke=0, ksg=0,
                         Ct=0, kda=.03, Comega=0, ksc=runif(1))),
  start = matrix(c(50, 0), ncol=1), 
  goal  = matrix(c(-500, 0), ncol=1),
  option0 = matrix(c(-11, 3, 28), ncol=1),
  option1 = matrix(c(-10, 0, 10), ncol=1),
  initial_speed = 0, 
  initial_angle = NA,
  obstacles = matrix(NA, ncol=1),
  collision_distance = 2, 
  steering_ratio = 5)
{
  out <- test_V2V(time, parameter, start, goal, option0, option1, 
                  initial_speed, initial_angle, obstacles, collision_distance,
                  steering_ratio)
  return (max(out$speeds))
}


##' @export
get_human_free_speed <- function(
  time = c(0, 15, .1, .5), 
  parameter = list(P = c(type = 0, kg=1, kc=.1, kdv=.45, ke=.5)),
  start = matrix(c(0, -5), ncol=1), 
  goal = matrix(c(0, 5), ncol=1),
  option0 = matrix(c(-1, .1, 1), ncol=1),
  option1 = matrix(c(-10, 0, 10), ncol=1),
  initial_speed = 0, 
  initial_angle = NA,
  obstacles = matrix(NA, ncol=1),
  collision_distance = 2, 
  steering_ratio = 5,
  wall = NA)
{
  out <- test_P2P(time, parameter, start, goal, option0, option1, initial_speed,
                  initial_angle, obstacles, collision_distance, steering_ratio,
                  wall)
  return (max(out$speeds))
}



######### Traffic tools  ---------------------------------------------------
##' Is on a Collision Course
##' 
##' Is on a Collision Course using the output from Simulation and Kinmatics 
##' class
##' 
##' @param obj an output from test_Run
##' @export
is_collision <- function(obj)
{
  if(length(obj$collision_distance) != 2) stop("Must be 2 agents")
  
  dx <- obj$x[2,] - obj$x[1,]  ## Agent 2 - Agent 1
  dy <- obj$y[2,] - obj$y[1,]
  distance <- sqrt(dx^2 + dy^2)
  return( list( isc = distance < obj$collision_distance[1],
                D   = distance) )
}



##' @export
find_tsee <- function(obj, wall = NULL, verbose = FALSE) 
{
  if( is.null(wall) ) stop("Must provide wall")
  nt   <- nrow(obj$times)
  blocked <- rep(FALSE, nt)
  
  for(i in 1:nt) {
    segment0 <- c(obj$x[1, i], obj$y[1, i], obj$x[2, i], obj$y[2, i])
    if(is_cross(segment0 = segment0, segment1 = wall)) blocked[i] <- TRUE
  }
  ## Because all blocked time point became 1 and the default is 0, the first
  ## minimal value (0) is the position immediately both agents saw each other.
  i_tsee <- which.min(blocked) 
  tsee <- obj$times[i_tsee, 1]
  
  ## Distance to min point
  D2M <- rep(NA, 2)
  D2M[1] <- norm( matrix(c(obj$x[1, i_tsee], 0, obj$y[1, i_tsee], 0), ncol=2), "F")
  D2M[2] <- norm( matrix(c(obj$x[2, i_tsee], 0, obj$y[2, i_tsee], 0), ncol=2), "F")
  
  t2M <- D2M / obj$speeds[, i_tsee] 
  
  lead_agent <- NA
  
  if ( t2M[1] < t2M[2] ) {
    if (verbose) cat("A0 is the leading agent\n")
    lead_agent <- 'A0'
  } else if ( t2M[1] == t2M[2] ) {
    if (verbose) cat("Equal time to midpoint\n")
  } else if (t2M[1] > t2M[2]) {
    if (verbose) cat("A1 is the leading agent\n")
    lead_agent <- 'A1'
  } else {
    if (verbose) cat("Unexpected situation.\n")
  }
  
  return( list(tsee=tsee, i_tsee=i_tsee, t2M=t2M, D2M=D2M,
               lead_agent=lead_agent) )
}

##' Find first possible collision point 
##' 
##' Test whether two line segments cross each other.
##' 
##' @param obj a list storing trajectory (x, y), speed, heading angles, and 
##' time stamps from a simulation run 
##' @param collision_distance a small number considering a collision is imminent 
##' 
##' @return a list of three elements. 
##' @export
find_collision_point <- function(obj, collision_distance = 1) {
  nt <- length(obj$times[,1])
  delta_x_traj <- obj$x[2,] - obj$x[1,]
  delta_y_traj <- obj$y[2,] - obj$y[1,]
  distance_traj <- sqrt(delta_x_traj^2+ delta_y_traj^2)
  is_colliding_traj <- distance_traj < collision_distance
  index <- NA
  
  for(i in 1:nt) {
    if (is_colliding_traj[i]) {
      index <- i
      cat("Find first possible collision point at", obj$times[i,1], "s")
      break
    }
  }
  cat("\n")
  
  collision_agent0 <- c(obj$x[1, index], obj$y[1, index])
  collision_agent1 <- c(obj$x[2, index], obj$y[2, index])
  da0 <- sqrt( sum( (obj$goal[1,] - collision_agent0)^2))
  da1 <- sqrt( sum( (obj$goal[2,] - collision_agent1)^2))
  
  if (is.na(index))
  {
    cat("No collision happens\n")
  } else if ( da0 < da1)
  {
    cat("A0 pasts first: ")
  } else 
  {
    cat("A1 pasts first: ")
  }
  cat("(Distance2Goal, A0 vs A1):", round(da0, 2), " ", round(da1,2), "\n")
  
  return(list(index=index, is_colliding_traj=is_colliding_traj,
              D2G = c(da0, da1)))
}

##' @export
who_pass_first <- function(obj, cross_point = c(0,0), verbose = FALSE) 
{
  nt <- nrow(obj$times)
  A0passed <- A1passed <- rep(FALSE, nt)
  for(i in 1:nt) 
  {
    A0passed[i] <- obj$x[1, i] > cross_point[1] && obj$y[1, i] > cross_point[2]
    A1passed[i] <- obj$x[2, i] > cross_point[1] && obj$y[2, i] < cross_point[2]
  }
  
  i_A0passed <- which.max(A0passed)
  i_A1passed <- which.max(A1passed)
  who_passed <- NA
  
  if (sum(A0passed, A1passed) == 0) {
    if (verbose) cat("Neither A0 nor A1 passed midpoint\n")
  } else if (sum(A0passed)==0) {
    if (verbose) cat("A0 never passed midpoint\n")
  } else if (sum(A1passed)==0) {
    if (verbose) cat("A1 never passed midpoint\n")
  } else if (i_A0passed < i_A1passed) {
    if (verbose) cat("A0 passed earlier\n")
    who_passed <- "A0"
  } else if (i_A0passed == i_A1passed) {
    if (verbose) cat("Both passed at the same time\n")
    who_passed <- "Both"
  } else if (i_A0passed > i_A1passed) {
    if (verbose) cat("A1 passed earlier\n")
    who_passed <- "A1"
  } else {
    if (verbose) cat("Unexpected situation\n")
  }
  
  list(cbind(A0passed, A1passed), who_passed=who_passed)
  
}

##' @export
get_speed_list <- function(x0, x1=NULL) {
  
  ns0 <- ns1 <- length(x0)
  if (!is.null(x1)) {
    ns1 <- length(x1)
  } else {
    x1 <- x0
  }
  
  out <- matrix(numeric(2*ns0*ns1), ncol=2)
  
  for(i in 0:(ns0-1)) {
    for(j in 0:(ns1-1)) {
      k <- j + i * ns1
      out[k+1, ] <- c(x0[i+1], x1[j+1]) 
    }
  }
  return(out)
}

