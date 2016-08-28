Altitude Heading Reference System (AHRS) Package
---

ROS node for data acquisition of pose and velocity information from the IG-500n device. The package is build around the SbgCom library provided by the vendor. 

The package could be extended to support other devices. It is implemented as a factory so any class implementing the ARHS interface could replace the current active class. 

The user is not aware of which device is used.
