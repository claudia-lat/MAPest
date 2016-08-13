#### Note for 2dofBowingTask

The code written here is the first attemps to solve inverse dynamics in human with MAP estimation by using the iDynTree library.
Clearly it is strongly experiment-dependent and not yet flexible for other kind of experiments.

For a better understanding of the URDF model used for this experiment go to the related paper in http://www.mdpi.com/1424-8220/16/5/727.  It is worth noting that while in the paper we used the angular-linear notation of Featherstone's spatial algebra (for more info see http://royfeatherstone.org/spatial/), in this repository it is adopted the linear-angular one.