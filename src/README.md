# Source Folder
This contains the primary source code for the quadrotor. It is divided into these different directories:
- `common` C++ code that is common between the Arduino and base station scripts. Contains most of the core computations run onboard, such 
as the LQR controller, estimator, and messaging protocols.
- `firmware` Arduino scripts for the Feathers
- `mocap` C++ code for processing MOCAP data (through the NatNet SDK) to the base station radio Feather M0 over serial
- `simulator` Julia and C++ code the WIP hardware-in-the-loop simulator
- `utils` Generally useful C++ code
