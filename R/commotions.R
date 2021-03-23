######### PACKAGE ---------------------------------------------------
##' A traffic model to simulate road user behaviours, including pedestrians
##' and drivers (vehicles). 
##'
##' @keywords package
##'
##' @name commotions
##' @docType package
##' @author  Yi-Shin Lin <yishinlin001@gmail.com>, Gustav Markkula <G.Markkula@leeds.ac.uk> 
##' @importFrom Rcpp evalCpp
##' @useDynLib commotions
NULL

######### System tools  ----------------------------------------------
##' @export
get_os <- function () {
  sysinf <- Sys.info()
  if (!is.null(sysinf))
  {
    os <- sysinf['sysname']
    if (os == 'Darwin') os <- "osx"
  } else 
  { ## mystery machine
    os <- .Platform$OS.type
    if ( grepl("^darwin", R.version$os) ) os <- "osx"
    if ( grepl("linux-gnu", R.version$os) ) os <- "linux"
  }
  tolower(os)
}

