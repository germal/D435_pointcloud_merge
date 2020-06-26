# vmrs

To build library,

`mkdir build; cd build`
`cmake ..`
`make -j

`TEST_rsd4xx.cc` contains the most recently successful code

`TEST_showPointCloud.cc` contains code being developed at the moment

Multiple realsense cameras can be coordinated at the moment.

* Each realsense camera is an instance of class RsD4xx
* Each instance has to call `init` and `proceed` separately
* Theoretically, all the cameras have to call `init` first
  * But if all `init` functions are called first, `Device or resource busy` error occurs
  * Therefore, each camera called `proceed` just once after being initialized
    * This way, all cameras proceed ONCE after initialization
    * Somehow prevents the error from occurring
* Theoretically, initializing cameras in order (camera0->camera1...) should work
  * Does not
  * Initializing the cameras in a backward order works instead
* Now preparing to merge the images into one

# TODO

* Leave commments where necessary
* Create a function (in the code itself) for matrix transformation using extrinsics
  * In the form of combine(mat1, mat2, R, T)
* ~~Designate the standard camera (identity transformation) by its serial number~~
  * ~~Therefore, the extrinsics must be based on the cameras' serial numbers.~~
* Get configgurations from YAML
  * Currently hardcoded